# Support for the Prusa MMU2S in usb peripheral mode
#
# Copyright (C) 2019  Eric Callahan <arksine.code@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.

import os
import subprocess
import logging
import serial
import threading
import filament_switch_sensor

MMU2_BAUD = 115200
RESPONSE_TIMEOUT = 45.

MMU_COMMANDS = {
    "SET_TOOL": "T%d",
    "LOAD_FILAMENT": "L%d",
    "SET_TMC_MODE": "M%d",
    "UNLOAD_FILAMENT": "U%d",
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
    FINDA_REFRESH_TIME = .3
    def __init__(self, config, mmu):
        super(FindaSensor, self).__init__(config)
        self.name = "finda"
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.mmu = mmu
        gcode_macro = self.printer.try_load_module(config, 'gcode_macro')
        self.runout_gcode = gcode_macro.load_template(
            config, 'runout_gcode', FINDA_GCODE)
        self.last_state = False
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
        try:
            self.last_state = int(self.mmu.send_command("READ_FINDA")[:-2])
        except self.gcode.error:
            logging.exception("mmu2s: error reading Finda, cannot initialize")
            return False
        waketime = self.reactor.monotonic() + self.FINDA_REFRESH_TIME
        self.reactor.update_timer(self.query_timer, waketime)
        return True
    def stop_query(self):
        self.reactor.update_timer(self.query_timer, self.reactor.NEVER)
    def _finda_event(self, eventtime):
        try:
            finda_val = int(self.mmu.send_command("READ_FINDA")[:-2])
        except self.gcode.error:
            logging.exception("mmu2s: error reading Finda, stopping timer")
            return self.reactor.NEVER
        if finda_val == self.last_state:
            return
        if not finda_val:
            # transition from filament present to not present
            if (self.runout_enabled and self.sensor_enabled and
                    (eventtime - self.last_event_time) > self.EVENT_DELAY):
                # Filament runout detected
                self.last_event_time = eventtime
                logging.info(
                    "switch_sensor: runout event detected, Time %.2f",
                    eventtime)
                self.reactor.register_callback(self._runout_event_handler)
        self.last_state = finda_val
        return eventtime + self.FINDA_REFRESH_TIME
    def cmd_QUERY_FILAMENT_SENSOR(self, params):
        if self.last_state:
            msg = "Finda: filament detected"
        else:
            msg = "Finda: filament not detected"
        self.gcode.respond_info(msg)
    def cmd_SET_FILAMENT_SENSOR(self, params):
        self.sensor_enabled = self.gcode.get_int("ENABLE", params, 1)

class IdlerSensor:
    def __init__(self, config):
        pin = config.get('idler_sensor_pin')
        printer = config.get_printer()
        buttons = printer.try_load_module(config, 'buttons')
        buttons.register_buttons([pin], self._button_handler)
        self.last_state = False
    def _button_handler(self, eventtime, status):
        self.last_state = status
    def get_idler_state(self):
        return self.last_state

class MMU2Serial:
    def __init__(self, config, resp_callback):
        self.port = config.get('serial', None)
        self.autodetect = self.port is None
        printer = config.get_printer()
        self.reactor = printer.get_reactor()
        self.gcode = printer.lookup_object('gcode')
        self.ser = None
        self.connected = False
        self.mmu_response = None
        self.mutex = self.reactor.mutex()
        self.response_cb = resp_callback
        self.partial_response = ""
        self.fd_handle = self.fd = None
    def connect(self, eventtime):
        logging.info("Starting MMU2S connect")
        if self.autodetect:
            self.port = detect_mmu_port()
            if self.port is None:
                raise self.gcode.error(
                    "mmu2s: Unable to autodetect serial port for MMU device")
        if not self._wait_for_program():
            raise self.gcode.error("mmu2s: unable to find mmu2s device")
        start_time = self.reactor.monotonic()
        while 1:
            connect_time = self.reactor.monotonic()
            if connect_time > start_time + 90.:
                raise self.gcode.error("mmu2s: Unable to connect to MMU2s")
            try:
                self.ser = serial.Serial(
                    self.port, MMU2_BAUD, stopbits=serial.STOPBITS_TWO,
                    timeout=0, exclusive=True)
            except (OSError, IOError, serial.SerialException) as e:
                logging.warn("Unable to MMU2S port: %s", e)
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
            if status is True:
                if not logged:
                    logging.info("mmu2s: Waiting to exit bootloader")
                    logged = True
                break
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
                    self.mmu_response = line
                    ack_count += 1
                else:
                    # Transfer initiated by MMU
                    self.response_cb(line)
            if ack_count > 1:
                logging.warn("mmu2s: multiple acknowledgements recd")
    def send_with_response(self, data, timeout=RESPONSE_TIMEOUT, retries=2):
        with self.mutex:
            if not self.connected:
                raise self.gcode.error(
                    "mmu2s: mmu disconnected, cannot send command %s" %
                    str(data[:-1]))
            self.mmu_response = None
            while retries:
                try:
                    self.ser.write(data)
                except serial.SerialException:
                    logging.warn("MMU2S disconnected")
                    self.disconnect()
                curtime = self.reactor.monotonic()
                last_resp_time = curtime
                endtime = curtime + timeout
                while self.mmu_response is None:
                    if not self.connected:
                        raise self.gcode.error(
                            "mmu2s: mmu disconnected, cannot send command %s" %
                            str(data[:-1]))
                    if curtime >= endtime:
                        break
                    curtime = self.reactor.pause(curtime + .01)
                    if curtime - last_resp_time >= 2.:
                        self.gcode.respond_info(
                            "mmu2s: waiting for response, %.2fs remaining" %
                            (endtime - curtime))
                        last_resp_time = curtime
                else:
                    # command acknowledge, response recd
                    resp = self.mmu_response
                    self.mmu_response = None
                    return resp
                retries -= 1
                if retries:
                    self.gcode.respond_info(
                        "mmu2s: retrying command %s" % (str(data[:-1])))
            raise self.gcode.error(
                "mmu2s: no acknowledgment for command %s" %
                (str(data[:-1])))

class MMU2S:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.gcode = self.printer.lookup_object('gcode')

        # Setup hardware reset pin
        ppins = self.printer.lookup_object('pins')
        self.reset_pin = ppins.setup_pin(
            'digital_out', config.get('reset_pin'))
        self.reset_pin.setup_max_duration(0.)
        self.reset_pin.setup_start_value(1, 1)
        self.mmu_serial = MMU2Serial(config, self._response_cb)
        self.finda = FindaSensor(config, self)
        self.ir_sensor = IdlerSensor(config)
        self.mmu_ready = False
        self.current_extruder = 0
        for t_cmd in ["Tx, Tc, T?"]:
            self.gcode.register_command(t_cmd, self.cmd_T_SPECIAL)
        self.gcode.register_command(
            "MMU_GET_STATUS", self.cmd_MMU_GET_STATUS)
        self.gcode.register_command(
            "MMU_SET_STEALTH", self.cmd_MMU_SET_STEALTH)
        self.gcode.register_command(
            "MMU_READ_IR", self.cmd_MMU_READ_IR)
        self.gcode.register_command(
            "MMU_FLASH_FIRMWARE", self.cmd_MMU_FLASH_FIRMWARE)
        self.gcode.register_command(
            "MMU_RESET", self.cmd_MMU_RESET)
        self.printer.register_event_handler(
            "klippy:ready", self._handle_ready)
        self.printer.register_event_handler(
            "klippy:disconnect", self._handle_disconnect)
        self.printer.register_event_handler(
            "gcode:request_restart", self._handle_disconnect)
    def send_command(self, cmd, reqtype=None):
        if cmd not in MMU_COMMANDS:
            raise self.gcode.error("mmu2s: Unknown MMU Command %s" % (cmd))
        command = MMU_COMMANDS[cmd]
        if reqtype is not None:
            command = command % (reqtype)
        outbytes = bytes(command + '\n')
        if 'P' in command:
            timeout = 3.
        else:
            timeout = RESPONSE_TIMEOUT
        try:
            resp = self.mmu_serial.send_with_response(
                outbytes, timeout=timeout)
        except self.gcode.error:
            self.mmu_ready = False
            raise
        return resp
    def _handle_ready(self):
        reactor = self.printer.get_reactor()
        self._hardware_reset()
        connect_time = reactor.monotonic() + 5.
        reactor.register_callback(self.mmu_serial.connect, connect_time)
    def _handle_disconnect(self, print_time=0.):
        self.mmu_ready = False
        self.finda.stop_query()
        self.mmu_serial.disconnect()
    def _hardware_reset(self):
        toolhead = self.printer.lookup_object('toolhead')
        print_time = toolhead.get_last_move_time()
        self.reset_pin.set_digital(print_time, 0)
        print_time = max(print_time + .1, toolhead.get_last_move_time())
        self.reset_pin.set_digital(print_time, 1)
    def _response_cb(self, data):
        if data == "start":
            self.mmu_ready = self.finda.start_query()
            if self.mmu_ready:
                self.gcode.respond_info("mmu2s: mmu ready for commands")
        else:
            self.gcode.respond_info(
                "mmu2s: unknown transfer from mmu\n%s" % data)
    def change_tool(self, index):
        pass
    def cmd_T_SPECIAL(self, params):
        # Hand T commands followed by special characters (x, c, ?)
        cmd = params['#command'].upper()
        if 'X' in cmd:
            pass
        elif 'C' in cmd:
            pass
        elif '?' in cmd:
            pass
    def cmd_MMU_GET_STATUS(self, params):
        ack = self.send_command("CHECK_ACK")
        version = self.send_command("GET_VERSION")[:-2]
        build = self.send_command("GET_BUILD_NUMBER")[:-2]
        errors = self.send_command("GET_DRIVE_ERRORS")[:-2]
        status = ("MMU Status:\nAcknowledge Test: %s\nVersion: %s\n" +
                  "Build Number: %s\nDrive Errors:%s\n")
        self.gcode.respond_info(status % (ack, version, build, errors))
    def cmd_MMU_SET_STEALTH(self, params):
        mode = self.gcode.get_int('MODE', params, minval=0, maxval=1)
        self.send_command("SET_TMC_MODE", mode)
    def cmd_MMU_READ_IR(self, params):
        ir_status = int(self.ir_sensor.get_idler_state())
        self.gcode.respond_info("mmu2s: IR Sensor Status = [%d]" % ir_status)
    def cmd_MMU_RESET(self, params):
        self._handle_disconnect()
        reactor = self.printer.get_reactor()
        # Give 5 seconds for the device reset
        self._hardware_reset()
        connect_time = reactor.monotonic() + 5.
        reactor.register_callback(self.mmu_serial.connect, connect_time)
    def cmd_MMU_FLASH_FIRMWARE(self, params):
        # XXX = get the toolhead object and make sure we arent
        # printing before proceeding
        reactor = self.printer.get_reactor()
        toolhead = self.printer.lookup_object('toolhead')
        if toolhead.get_status(reactor.monotonic())['status'] == "Printing":
            self.gcode.respond_info(
                "mmu2s: cannot update firmware while printing")
            return
        avrd_cmd = ["avrdude", "-p", " atmega32u4", "-c", "avr109"]
        fname = self.gcode.get_str("FILE", params)
        if fname[-3:] != "hex":
            self.gcode.respond_info(
                "mmu2s: File does not appear to be a valid hex: %s" % (fname))
            return
        if fname.startswith('~'):
            fname = os.path.expanduser(fname)
        if os.path.exists(fname):
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
            self._handle_disconnect()
            self._hardware_reset()
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


def load_config(config):
    return MMU2S(config)
