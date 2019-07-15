# Support for the Prusa MMU2S in usb peripheral mode
#
# Copyright (C) 2019  Eric Callahan <arksine.code@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.

import os
import subprocess
import logging
import serial
import filament_switch_sensor

MMU2_BAUD = 115200
MMU_TIMEOUT = 45.

MMU_COMMANDS = {
    "SET_TOOL": "T%d",
    "LOAD_FILAMENT": "L%d",
    "SET_TMC_MODE": "M%d",
    "UNLOAD_FILAMENT": "U0",
    "RESET": "X0",
    "READ_FINDA": "P0",
    "CHECK_ACK": "S0",
    "GET_VERSION": "S1",
    "GET_BUILD_NUMBER": "S2",
    "GET_DRIVE_ERRORS": "S3",
    "SET_FILAMENT": "F%d",  # This appears to be a placeholder, does nothing
    "CONTINUE_LOAD": "C0",  # Load to printer gears
    "EJECT_FILAMENT": "E%d",
    "RECOVER": "R0",        # Recover after eject
    "WAIT_FOR_USER": "W0",
    "CUT_FILAMENT": "K0"
}

# Run command helper function allows stdout to be redirected
# to the ptty without its file descriptor.
def run_command(command):
    p = subprocess.Popen(command,
                         stdout=subprocess.PIPE,
                         stderr=subprocess.STDOUT)
    return iter(p.stdout.readline, b'')

# USB Device Helper Functions

# Checks to see if device is in bootloader mode
# Returns True if in bootloader mode, False if in device mode
# and None if no device is detected
def check_bootloader(portname):
    ttyname = os.path.realpath(portname)
    for fname in os.listdir('/dev/serial/by-id/'):
        fname = '/dev/serial/by-id/' + fname
        if os.path.realpath(fname) == ttyname and \
                "Multi_Material" in fname:
            return "bootloader" in fname
    return None

# Attempts to detect the serial port for a connected MMU device.
# Returns the device name by usb path if device is found, None
# if no device is found. Note that this isn't reliable if multiple
# mmu devices are connected via USB.
def detect_mmu_port():
    for fname in os.listdir('/dev/serial/by-id/'):
        if "MK3_Multi_Material_2.0" in fname:
            fname = '/dev/serial/by-id/' + fname
            realname = os.path.realpath(fname)
            for fname in os.listdir('/dev/serial/by-path/'):
                fname = '/dev/serial/by-path/' + fname
                if realname == os.path.realpath(fname):
                    return fname
    return None

# XXX - The current gcode is temporary.  Need to determine
# the appropriate action the printer and MMU should take
# on a finda runout, then execute the appropriate gcode.
# I suspect it is some form of M600
FINDA_GCODE = '''
M118 Finda Runout Detected
M117 Finda Runout Detected
'''

class FindaSensor(filament_switch_sensor.BaseSensor):
    EVENT_DELAY = 3.
    FINDA_RETRY_TIME = 3.
    FINDA_REFRESH_TIME = .3
    def __init__(self, config, mmu):
        super(FindaSensor, self).__init__(config)
        self.name = "finda"
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.mmu = mmu
        self.finda_cmd = self.mmu.get_cmd("READ_FINDA")
        gcode_macro = self.printer.try_load_module(config, 'gcode_macro')
        self.runout_gcode = gcode_macro.load_template(
            config, 'runout_gcode', FINDA_GCODE)
        self.last_state = 0
        self.last_event_time = 0.
        self.query_timer = self.reactor.register_timer(self._finda_event)
        self.sensor_enabled = True
        self.gcode.register_mux_command(
            "QUERY_FILAMENT_SENSOR", "SENSOR", self.name,
            self.cmd_QUERY_FILAMENT_SENSOR,
            desc=self.cmd_QUERY_FILAMENT_SENSOR_help)
        self.gcode.register_mux_command(
            "SET_FILAMENT_SENSOR", "SENSOR", self.name,
            self.cmd_SET_FILAMENT_SENSOR,
            desc=self.cmd_SET_FILAMENT_SENSOR_help)
    def start_query(self):
        self.last_state = self.mmu.send_command(
            self.finda_cmd, timeout=3.)
        if self.last_state < 0:
            logging.info("mmu2s: error reading Finda, cannot initialize")
            return False
        waketime = self.reactor.monotonic() + self.FINDA_REFRESH_TIME
        self.reactor.update_timer(self.query_timer, waketime)
        return True
    def stop_query(self):
        self.reactor.update_timer(self.query_timer, self.reactor.NEVER)
    def _finda_event(self, eventtime):
        finda_val = self.mmu.send_command(
            self.finda_cmd, timeout=3.)
        # There is a roundtrip delay when performing reads, so fetch
        # current monotonic time
        curtime = self.reactor.monotonic()
        if finda_val < 0:
            # Error retreiving finda, try again in 3 seconds
            return curtime + self.FINDA_RETRY_TIME
        if finda_val != self.last_state:
            # Transition of state, check value
            if not finda_val:
                # transition from filament present to not present
                if (self.runout_enabled and self.sensor_enabled and
                        (curtime - self.last_event_time) > self.EVENT_DELAY):
                    # Filament runout detected
                    self.last_event_time = curtime
                    self.reactor.register_callback(self._runout_event_handler)
        self.last_state = finda_val
        return curtime + self.FINDA_REFRESH_TIME
    def cmd_QUERY_FILAMENT_SENSOR(self, params):
        if self.last_state:
            msg = "Finda: filament detected"
        else:
            msg = "Finda: filament not detected"
        self.gcode.respond_info(msg)
    def cmd_SET_FILAMENT_SENSOR(self, params):
        self.sensor_enabled = self.gcode.get_int("ENABLE", params, 1)

class IdlerSensor:
    def __init__(self, config, mmu2s):
        pin = config.get('idler_sensor_pin')
        printer = config.get_printer()
        buttons = printer.try_load_module(config, 'buttons')
        buttons.register_buttons([pin], self._button_handler)
        self.mmu2s = mmu2s
        self.last_state = False
    def _button_handler(self, eventtime, status):
        self.last_state = status
    def get_idler_state(self):
        return self.last_state

class MMU2Serial:
    DISCONNECT_MSG = "mmu2s: mmu disconnected, cannot send command %s"
    NACK_MSG = "mmu2s: no acknowledgment for command %s"
    def __init__(self, config, resp_callback):
        self.port = config.get('serial', None)
        self.autodetect = self.port is None
        printer = config.get_printer()
        self.reactor = printer.get_reactor()
        self.send_completion = None
        self.gcode = printer.lookup_object('gcode')
        self.ser = None
        self.connected = False
        self.response_cb = resp_callback
        self.partial_response = ""
        self.keepalive_timer = self.reactor.register_timer(
            self._keepalive_event)
        self.fd_handle = self.fd = None
    def connect(self, eventtime):
        logging.info("Starting MMU2S connect")
        if self.autodetect:
            self.port = detect_mmu_port()
            if self.port is None:
                logging.info(
                    "mmu2s: Unable to autodetect serial port for MMU device")
                return
        if not self._wait_for_program():
            logging.info("mmu2s: unable to find mmu2s device")
            return
        start_time = self.reactor.monotonic()
        while 1:
            connect_time = self.reactor.monotonic()
            if connect_time > start_time + 90.:
                # Give 90 second timeout, then raise error.
                raise self.gcode.error("mmu2s: Unable to connect to MMU2s")
            try:
                self.ser = serial.Serial(
                    self.port, MMU2_BAUD, stopbits=serial.STOPBITS_TWO,
                    timeout=0, exclusive=True)
            except (OSError, IOError, serial.SerialException) as e:
                logging.exception("Unable to MMU2S port: %s", e)
                self.reactor.pause(connect_time + 5.)
                continue
            break
        self.connected = True
        self.fd = self.ser.fileno()
        self.fd_handle = self.reactor.register_fd(
            self.fd, self._handle_mmu_recd)
        logging.info("MMU2S connected")
    def _wait_for_program(self):
        # Waits until the device program is loaded, pausing
        # if bootloader is detected
        timeout = 10.
        pause_time = .1
        logged = False
        while timeout > 0.:
            status = check_bootloader(self.port)
            if status is True and not logged:
                logging.info("mmu2s: Waiting to exit bootloader")
                logged = True
            elif status is False:
                logging.info("mmu2s: Device found on %s" % self.port)
                return True
            self.reactor.pause(self.reactor.monotonic() + pause_time)
            timeout -= pause_time
        logging.info("mmu2s: No device detected")
        return False
    def disconnect(self):
        if self.connected:
            if self.fd_handle is not None:
                self.reactor.unregister_fd(self.fd_handle)
            if self.ser is not None:
                self.ser.close()
                self.ser = None
            self.connected = False
    def _handle_mmu_recd(self, eventtime):
        try:
            data = self.ser.read(64)
        except serial.SerialException as e:
            logging.warn("MMU2S disconnected\n" + str(e))
            self.disconnect()
        if self.connected and data:
            lines = data.split('\n')
            lines[0] = self.partial_response + lines[0]
            self.partial_response = lines.pop()
            ack_count = 0
            for line in lines:
                if "ok" in line:
                    # acknowledgement
                    if self.send_completion is not None:
                        self.send_completion.complete(line)
                    else:
                        # acknowledgement recevied without request
                        self.response_cb(line)
                    ack_count += 1
                else:
                    # Transfer initiated by MMU
                    self.response_cb(line)
            if ack_count > 1:
                logging.warn("mmu2s: multiple acknowledgements recd")
    def send(self, data):
        if self.connected:
            try:
                self.ser.write(data)
            except serial.SerialException:
                logging.warn("MMU2S disconnected")
                self.disconnect()
        else:
            self.error_msg = self.DISCONNECT_MSG % str(data[:-1])
    def send_with_response(self, data, timeout=MMU_TIMEOUT,
                           retries=0, keepalive=True):
        # Sends data and waits for acknowledgement.  Returns a tuple,
        # The first value a boolean indicating success, the second is
        # the payload if successful, or an error message if the request
        # failed
        if not self.connected:
            return False, self.DISCONNECT_MSG % str(data[:-1])
        self.mmu_response = None
        while True:
            self.send_completion = self.reactor.completion()
            try:
                self.ser.write(data)
            except serial.SerialException:
                logging.warn("MMU2S disconnected")
                self.disconnect()
                return False, self.DISCONNECT_MSG % str(data[:-1])
            if keepalive:
                self.reactor.update_timer(
                    self.keepalive_timer, self.reactor.monotonic() + 2.)
            result = self.send_completion.wait(
                self.reactor.monotonic() + timeout)
            self.reactor.update_timer(self.keepalive_timer, self.reactor.NEVER)
            self.send_completion = None
            if result is not None:
                return True, result
            if retries:
                retries -= 1
                self.gcode.respond_info(
                    "mmu2s: retrying command %s" % (str(data[:-1])))
            else:
                break
        return False, self.NACK_MSG % str(data[:-1])
    def wait_for_user(self, keepalive=True):
        if not self.connected:
            return False
        self.send_completion = self.reactor.completion()
        if keepalive:
            self.reactor.update_timer(
                self.keepalive_timer, self.reactor.monotonic() + 2.)
        result = self.send_completion.wait()
        self.reactor.update_timer(self.keepalive_timer, self.reactor.NEVER)
        self.send_completion = None
        return result != "ABORT"
    def action_abort_wait(self):
        if self.send_completion is not None:
            self.send_completion.complete("ABORT")
        return ""
    def _keepalive_event(self, eventtime):
        self.gcode.respond_info(
            "mmu2s: waiting for command acknowledgement")
        return eventtime + 2.

# Handles load/store to local persistent storage
class MMUStorage:
    def __init__(self):
        pass

# Handles display interaction for MMU prompts
class MMUDisplay:
    def __init__(self):
        pass

class MMU2USBControl:
    def __init__(self, config, mmu):
        self.printer = config.get_printer()
        self.gcode = self.printer.lookup_object('gcode')
        self.mmu = mmu
        self.mmu_serial = mmu.mmu_serial
        ppins = self.printer.lookup_object('pins')
        self.reset_pin = ppins.setup_pin(
            'digital_out', config.get('reset_pin'))
        self.reset_pin.setup_max_duration(0.)
        self.reset_pin.setup_start_value(1, 1)
        self.gcode.register_command(
            "MMU_FLASH_FIRMWARE", self.cmd_MMU_FLASH_FIRMWARE)
        self.gcode.register_command(
            "MMU_RESET", self.cmd_MMU_RESET)
    def hardware_reset(self):
        toolhead = self.printer.lookup_object('toolhead')
        print_time = toolhead.get_last_move_time()
        self.reset_pin.set_digital(print_time, 0)
        print_time = max(print_time + .1, toolhead.get_last_move_time())
        self.reset_pin.set_digital(print_time, 1)
    def cmd_MMU_RESET(self, params):
        self.mmu.disconnect()
        reactor = self.printer.get_reactor()
        # Give 5 seconds for the device reset
        self.hardware_reset()
        connect_time = reactor.monotonic() + 5.
        reactor.register_callback(self.mmu_serial.connect, connect_time)
    def cmd_MMU_FLASH_FIRMWARE(self, params):
        reactor = self.printer.get_reactor()
        toolhead = self.printer.lookup_object('toolhead')
        if toolhead.get_status(reactor.monotonic())['status'] == "Printing":
            self.gcode.respond_info(
                "mmu2s: cannot update firmware while printing")
            return
        avrd_cmd = ["avrdude", "-p", "atmega32u4", "-c", "avr109"]
        fname = self.gcode.get_str("FILE", params)
        if fname[-4:] != ".hex":
            self.gcode.respond_info(
                "mmu2s: File does not appear to be a valid hex: %s" % (fname))
            return
        if fname.startswith('~'):
            fname = os.path.expanduser(fname)
        if os.path.exists(fname):
            # Firmware file found, attempt to locate MMU2S port and flash
            if self.mmu_serial.autodetect:
                port = detect_mmu_port()
            else:
                port = self.mmu_serial.port
            try:
                ttyname = os.path.realpath(port)
            except:
                self.gcode.respond_info(
                    "mmu2s: unable to find mmu2s device on port: %s" % (port))
                return
            avrd_cmd += ["-P", ttyname, "-D", "-U", "flash:w:%s:i" % (fname)]
            self.mmu.disconnect()
            self.hardware_reset()
            timeout = 5
            while timeout:
                reactor.pause(reactor.monotonic() + 1.)
                if check_bootloader(port):
                    # Bootloader found, run avrdude
                    for line in run_command(avrd_cmd):
                        self.gcode.respond_info(line)
                    return
                timeout -= 1
            self.gcode.respond_info("mmu2s: unable to enter mmu2s bootloader")
        else:
            self.gcode.respond_info(
                "mmu2s: Cannot find firmware file: %s" % (fname))

# XXX - Class containing test gcodes for MMU, to be remove
class MMUTest:
    def __init__(self, mmu):
        self.send_command = mmu.send_command
        self.get_cmd = mmu.get_cmd
        self.gcode = mmu.gcode
        self.ir_sensor = mmu.ir_sensor
        self.gcode.register_command(
            "MMU_GET_STATUS", self.cmd_MMU_GET_STATUS)
        self.gcode.register_command(
            "MMU_SET_STEALTH", self.cmd_MMU_SET_STEALTH)
        self.gcode.register_command(
            "MMU_READ_IR", self.cmd_MMU_READ_IR)
    def cmd_MMU_GET_STATUS(self, params):
        cmds = ["CHECK_ACK", "GET_VERSION", "GET_BUILD_NUMBER",
                "GET_DRIVE_ERRORS"]
        cmds = [self.get_cmd(c) for c in cmds]
        responses = [self.send_command(c) for c in cmds]
        responses[0] = responses[0] == 0
        status = ("MMU Status:\nAcknowledge Test: %d\nVersion: %d\n" +
                  "Build Number: %d\nDrive Errors:%d\n")
        self.gcode.respond_info(status % tuple(responses))
    def cmd_MMU_SET_STEALTH(self, params):
        mode = self.gcode.get_int('MODE', params, minval=0, maxval=1)
        cmd = self.get_cmd("SET_TMC_MODE")
        self.send_command(cmd, mode)
    def cmd_MMU_READ_IR(self, params):
        ir_status = int(self.ir_sensor.get_idler_state())
        self.gcode.respond_info("mmu2s: IR Sensor Status = [%d]" % ir_status)

MAX_LOAD_MORE_ATTEMPTS = 21

class MMU2S:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.gcode = self.printer.lookup_object('gcode')
        self.cutter_enabled = config.getboolean('cutter_enabled', False)
        self.e_velocity = config.getfloat('mmu_load_velocity', 19.02, above=0.)
        self.mmu_load_time = config.getfloat('mmu_load_time', 2., above=0.)
        park_pos = config.get('park_position', "50,190,20")
        park_pos = park_pos.strip().split(',', 2)
        try:
            self.park_xyz = [float(p.strip()) for p in park_pos]
        except:
            raise config.error("Unable to parse park_position")
        zcfg = config.getsection('stepper_z')
        self.zmax = zcfg.get('position_max')
        self.mutex = self.printer.get_reactor().mutex()
        self.mmu_serial = MMU2Serial(config, self._mmu_serial_event)
        self.finda = FindaSensor(config, self)
        self.ir_sensor = IdlerSensor(config, self)
        self.mmu_usb_ctrl = MMU2USBControl(config, self)
        self.mmu_ready = False
        self.version = self.build_number = 0
        self.user_completion = None
        self.cmd_ack = False
        self.move_completion = None
        self.current_extruder = 0
        self.filament_loaded = False
        self.hotend_temp = 0.
        for t_cmd in ["Tx, Tc, T?"]:
            self.gcode.register_command(t_cmd, self.cmd_T_SPECIAL)
        self.printer.register_event_handler(
            "klippy:ready", self._handle_ready)
        self.printer.register_event_handler(
            "klippy:disconnect", self.disconnect)
        self.printer.register_event_handler(
            "gcode:request_restart", self.disconnect)
        self.printer.register_event_handler(
            "klippy:shutdown", self.disconnect)
        # XXX - testing object, to be removed
        MMUTest(self)
    def _mmu_serial_event(self, data):
        if data == "start":
            self.mmu_ready = self.finda.start_query()
            self.version = self.send_command(
                self.get_cmd("GET_VERSION"))
            self.build_number = self.send_command(
                self.get_cmd("GET_BUILD_NUMBER"))
            if self.mmu_ready:
                version = ".".join(str(self.version))
                self.gcode.respond_info(
                    "mmu2s: mmu ready, Firmware Version: %s Build Number: %d" %
                    (version, self.build_number))
            else:
                self.gcode.respond_info(
                    "mmu2s: unknown transfer from mmu\n%s" % data)
    def _handle_ready(self):
        self.toolhead = self.printer.lookup_object('toolhead')
        self.mmu_usb_ctrl.hardware_reset()
        connect_time = self.reactor.monotonic() + 5.
        self.reactor.register_callback(self.mmu_serial.connect, connect_time)
    def disconnect(self, print_time=0.):
        self.mmu_ready = False
        self.finda.stop_query()
        self.mmu_serial.disconnect()
    def get_cmd(self, cmd, reqtype=None):
        if cmd not in MMU_COMMANDS:
            raise self.gcode.error("mmu2s: Unknown MMU Command %s" % (cmd))
        command = MMU_COMMANDS[cmd]
        if reqtype is not None:
            command = command % (reqtype)
        return bytes(command + '\n')
    def send_command_async(self, cmd, timeout=MMU_TIMEOUT, retries=0):
        # Caller MUST acquire mutex
        def send(eventtime, o=cmd, t=timeout, r=retries):
            self.cmd_ack = False
            self.cmd_ack, data = self.mmu_serial.send_with_response(
                o, t, r)
            if self.move_completion is not None:
                self.move_completion.complete(data)
        self.reactor.register_callback(send)
    def send_command(self, cmd, timeout=MMU_TIMEOUT, retries=0):
        with self.mutex:
            self.cmd_ack = False
            self.cmd_ack, data = self.mmu_serial.send_with_response(
                cmd, timeout, retries)
            ret = 0
            if not self.cmd_ack:
                self.gcode.respond_info(data)
                ret = -1
            elif len(data) > 2:
                try:
                    ret = int(data[:-2])
                except:
                    ret = 0
            return ret
    def change_tool(self, index):
        # XXX - Check to see if autodeplete is enabled.  If so, get the next
        # available tool rather than using the supplied index
        if index == self.current_extruder:
            if self.filament_loaded:
                self.gcode.respond_info(
                    "Extruder%d already loaded" % index)
                return
        self.finda.stop_query()
        self.filament_loaded = True
        # XXX - Mark filament loaded for autodeplete
        if self.cutter_enabled:
            # XXX - Cut filament
            pass
        cmd = self.get_cmd("SET_TOOL", index)
        self.mmu_exec_cmd(cmd, retries=2)
        self.mmu_continue_loading(index)
        self.current_extruder = index
        self.finda.start_query()
    def mmu_cut_filament(self):
        pass
    def mmu_exec_cmd(self, cmd, unload=True, load=True,
                     park=True, hotend_off=True, retries=0):
        with self.mutex:
            error_recorded = False
            self.store_hotend_target()
            self.gcode.run_script_from_command(
                "SAVE_GCODE_STATE STATE=MMU_CMD_STATE")
            self.extruder_motor_off()
            self.send_command_async(cmd, retries=retries)
            self.mmu_extruder_move_sync(need_unload=unload, need_load=load)
            if not self.cmd_ack:
                error_recorded = True
                # XXX - save error to persistent memory
                if park:
                    self.park_extruder()
                if hotend_off:
                    self.gcode.run_script_from_command("M104 S0")
                self.extruder_motor_off()
                # XXX - Display message on lcd
                # XXX - It seems we need two types of delays here,
                # one that waits for a response from the lcd display
                # if we turned the hotend off, and the current one that
                # waits for a mmu button to be pressed.  I need to look
                # at the mmu code, but is suspect that actions that dont
                # require the nozzle at temperature automatically go into
                # wait mode
                if not self.mmu_serial.wait_for_user():
                    # need to raise an error and abort the print
                    raise self.gcode.error("MMU2S Error, print aborted")
            # XXX - check ok?  Check Finda?
            if error_recorded:
                self.gcode.run_script_from_command(
                    "M109 S%.4f" % (self.hotend_temp))
            self.gcode.run_script_from_command(
                "RESTORE_GCODE_STATE STATE=MMU_CMD_STATE MOVE=1 MOVE_SPEED=50")
    def mmu_extruder_move_sync(self, need_unload=True, need_load=True):
        need_unload = need_unload and not self.ir_sensor.get_idler_state()
        self.move_completion = self.reactor.completion()
        self.reactor.pause(self.reactor.monotonic() + .1)
        while not self.cmd_ack:
            if need_unload:
                if not self.extruder_unload_filament():
                    self.extruder_motor_off()
                    # Delay after unload is complete allowing
                    # tool to change
                    self.reactor.pause(
                        self.reactor.monotonic() + self.mmu_load_time)
                    need_unload = False
            elif need_load:
                if self.extruder_load_filament():
                    # Filament detected, stop loading and send
                    # abort signal
                    need_load = False
                    self.mmu_serial.send(b"A\n")
            else:
                break
        data = self.move_completion.wait()
        self.move_completion = None
        return data
    def extruder_load_filament(self):
        self.loading_in_progress = True
        self.gcode.run_script_from_command(
            "G1 E%.5f F%.5f" %
            (self.e_velocity * .1, self.e_velocity * 60))
        self.toolhead.wait_moves()
        return self.ir_sensor.get_idler_state()
    def extruder_unload_filament(self):
        self.gcode.run_script_from_command(
            "G1 E-%.5f F%.5f" %
            (self.e_velocity * 2., self.e_velocity * 60))
        self.toolhead.wait_moves()
        return self.ir_sensor.get_idler_state()
    def mmu_continue_loading(self, index):
        success = self.mmu_load_more()
        if success:
            success = self.mmu_can_load()
        if not success:
            # XXX increment persistent storage load failure
            pass
        attempts = 3
        while not success:
            if attempts:
                if self.cutter_enabled():
                    # XXX cut filament
                    pass
                cmd = self.get_cmd("SET_TOOL", index)
                self.mmu_exec_cmd(cmd)
                success = self.mmu_load_more()
                if success:
                    success = self.mmu_can_load()
                attempts -= 1
                continue
            self.gcode.run_script_from_command(
                "SAVE_GCODE_STATE STATE=MMU_CMD_STATE")
            # Unload MMU2S filament
            cmd = self.get_cmd("UNLOAD_FILAMENT")
            self.mmu_exec_cmd(cmd, load=False, park=False)
            # Pause, Park, notify, return control to user
            self.park_extruder()
            cmd = self.get_cmd("WAIT_FOR_USER")
            resp = self.send_command(cmd, timeout=self.reactor.NEVER)
            # XXX - send display / octoprint notification
            if resp == "ABORT":
                raise self.gcode.error("MMU2S Print Aborted By User")
            else:
                self.gcode.run_script_from_command(
                    "M109 S%.4f"
                    "RESTORE_GCODE_STATE STATE=MMU_CMD_STATE "
                    "MOVE=1 MOVE_SPEED=50\n" % (self.hotend_temp))
                attempts = 1
    def mmu_load_more(self):
        cmd = self.get_cmd("CONTINUE_LOAD")
        for i in range(MAX_LOAD_MORE_ATTEMPTS):
            if self.ir_sensor.get_idler_state():
                return True
            self.mmu_exec_cmd(cmd, unload=False)
        return False
    def mmu_can_load(self):
        # Move extruder forward 60 mm, backward 52, then check idler sensor
        e_feedrate = self.e_velocity * 60.
        self.gcode.run_script_from_command(
            "G1 E%.5f F%.5f\n"
            "G1 E-%.5f F%.5f" %
            (60., e_feedrate, 52., e_feedrate))
        self.toolhead.wait_moves()
        detect_count = 0
        increment = 0.2
        steps = 30
        for i in range(steps):
            self.gcode.run_script_from_command(
                "G1 E-%.2f F%.2f" %
                (increment, e_feedrate))
            self.toolhead.wait_moves()
            if self.ir_sensor.get_idler_state():
                detect_count += 1
        return detect_count > (steps - 4)
    def load_to_nozzle(self):
        pass
    def store_hotend_target(self):
        curtime = self.reactor.monotonic()
        extruder = self.toolhead.get_extruder()
        e_status = extruder.get_status(curtime)
        self.hotend_temp = e_status['target']
    def park_extruder(self):
        curtime = self.reactor.monotonic()
        gc_status = self.gcode.get_status(curtime)
        current_z = gc_status['gcode_position'].z
        if current_z + self.park_xyz[2] < self.zmax:
            self.gcode.run_script_from_command(
                "G91\n"
                "G1 Z%.5f F900\n"
                "G90\n"
                "G1 X%.5f Y%.5f F3000" % (
                    self.park_xyz[2], self.park_xyz[0], self.park_xyz[1]))
        else:
            self.gcode.run_script_from_command(
                "G1 Z%.5f F900\n"
                "G1 X%.5f Y%.5f F3000\n" % (
                    self.zmax, self.park_xyz[0], self.park_xyz[1]))
    def extruder_motor_off(self):
        extruder = self.toolhead.get_extruder()
        extruder.motor_off(self.toolhead.get_last_move_time())
        self.toolhead.wait_moves()
    def cmd_T_SPECIAL(self, params):
        # XXX - After a closer look at Prusa Firmware it seems like these T
        # gcodes may not be necessary.  They all do some form of partial
        # toolchange, presumably these are called not via gcode but via display
        #
        # Hand T commands followed by special characters (x, c, ?)
        cmd = params['#command'].upper()
        if 'X' in cmd:
            pass
        elif 'C' in cmd:
            pass
        elif '?' in cmd:
            pass


def load_config(config):
    return MMU2S(config)
