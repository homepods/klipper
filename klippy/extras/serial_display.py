# Serial-Uart (gcode) LCD display support
#
# Copyright (C) 2020  Eric Callahan <arksine.code@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import serial
import os
import time
import json
import logging
import tempfile
import util

MIN_EST_TIME = 10.

class SerialDisplay:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.gcode = self.printer.lookup_object('gcode')
        self.port = config.get('serial')
        self.baud = config.get('baud', 115200)
        source_choices = {'paneldue': PanelDue, 'bigtreetechtft': BTTDisplay}
        source_type = config.getchoice('type', source_choices)
        self.source = source_type(config, self)
        self.ser = self.fd = None
        self.connected = False
        self.fd_handle = None
        self.partial_input = ""
        self.printer.register_event_handler(
            "klippy:connect", self._connect)
        self.printer.register_event_handler(
            "klippy:disconnect", self._handle_disconnect)
        self.printer.register_event_handler(
            "gcode:respond", self.source.handle_gcode_response)
        # load display status so we can get M117 messages
        self.printer.load_object(config, 'display_status')

    def _handle_disconnect(self):
        self.source.handle_disconnect()
        self.disconnect()

    def disconnect(self):
        if self.connected:
            if self.fd_handle is not None:
                self.reactor.unregister_fd(self.fd_handle)
                self.fd_handle = None
            self.connected = False
            self.ser.close()
            self.ser = None

    def _connect(self):
        start_time = connect_time = self.reactor.monotonic()
        while not self.connected:
            if connect_time > start_time + 30.:
                logging.info("serial_display: Unable to connect, aborting")
                break
            try:
                # XXX - sometimes the port cannot be exclusively locked, this
                # would likely be due to a restart where the serial port was
                # not correctly closed.  Maybe don't use exclusive mode?
                self.ser = serial.Serial(
                    self.port, self.baud, timeout=0, exclusive=True)
            except (OSError, IOError, serial.SerialException) as e:
                logging.warn("serial_display: unable to open port: %s", e)
                connect_time = self.reactor.pause(connect_time + 2.)
                continue
            self.fd = self.ser.fileno()
            util.set_nonblock(self.fd)
            self.fd_handle = self.reactor.register_fd(
                self.fd, self._process_data)
            self.connected = True
            logging.info(
                "serial_display: Display %s connected" % self.source.name)

    def _process_data(self, eventtime):
        # Process incoming data using same method as gcode.py
        try:
            data = os.read(self.fd, 4096)
        except os.error:
            if self.printer.is_shutdown():
                logging.exception("serial_display: read error while shutdown")
                self.disconnect()
            return

        if not data:
            # possibly an error, disconnect
            self.disconnect()
            logging.info("serial_display: No data received, disconnecting")
            return

        # Remove null bytes, separate into lines
        data = data.strip('\x00')
        lines = data.split('\n')
        lines[0] = self.partial_input + lines[0]
        self.partial_input = lines.pop()
        for line in lines:
            line = line.strip()
            try:
                self.source.process_line(line)
            except Exception:
                logging.exception(
                    "serial_display: GCode Processing Error: " + line)
                self.source.handle_gcode_response(
                    "!! GCode Processing Error: " + line)

    def send(self, data):
        if self.connected:
            while data:
                try:
                    sent = os.write(self.fd, data)
                except os.error as e:
                    if e.errno == 9 or e.errno == 32:
                        sent = 0
                    else:
                        waketime = self.reactor.monotonic() + .001
                        self.reactor.pause(waketime)
                        continue
                if sent:
                    data = data[sent:]
                else:
                    logging.exception(
                        "serial_display: Error writing data,"
                        " closing serial connection")
                    self.disconnect()
                    return

    def emergency_shutdown(self):
        self.gcode.cmd_M112({})

class PanelDue:
    def __init__(self, config, display):
        self.name = "PanelDue"
        self.printer = config.get_printer()
        self.gcode = self.printer.lookup_object('gcode')
        self.reactor = self.printer.get_reactor()
        self.serial_display = display
        p_cfg = config.getsection('printer')
        self.kinematics = p_cfg.get('kinematics')
        self.machine_name = config.get('machine_name', "Klipper")
        self.last_message = None
        self.last_gcode_response = None
        self.current_file = ""
        self.file_metadata = {}

        # Get non-trivial responses from the configuration.  These are
        ntkeys = config.get('non_trivial_keys', "Klipper state")
        self.non_trivial_keys = [k for k in ntkeys.split('\n') if k.strip()]

        # HACK - Report as "Repetier" firmware.  The PanelDue
        # will implement workarounds that eliminate the need to
        # handle "G10" as M104 and standby temperatures
        start_args = self.printer.get_start_args()
        version = start_args['software_version']
        self.firmware_name = "Repetier | Klipper " + version

        # Get configured macros
        self.available_macros = {}
        macros = config.get('macros', None)
        if macros is None:
            # No macros configured, default to all gcode macros in printer.cfg
            macro_sections = config.get_prefix_sections('gcode_macro')
            for m in macro_sections:
                name = m.get_name().split()[-1].upper()
                self.available_macros[name] = name
        else:
            # The macro's configuration name is the key, whereas the full
            # command is the value
            macros = [m for m in macros.split('\n') if m.strip()]
            self.available_macros = {m.split()[0]: m for m in macros}

        self.printer_objects = {'gcode': self.gcode}
        self.extruder_count = 0
        self.is_ready = False
        self.printer.register_event_handler(
            "klippy:ready", self.handle_ready)

        # These commands must bypass the gcode queue.  They only request
        # state which updates the display, they do not change gcode state.
        # This is analogous to how the basic LCD display fetches printer
        # state via calls to get_status()
        self.direct_gcodes = {
            'M20': self._run_paneldue_M20,
            'M36': self._run_paneldue_M36,
            'M408': self._run_paneldue_M408
        }

        # These gcodes require special parsing or handling prior to being
        # run by Klipper's primary gcode handler
        self.special_gcodes = {
            'M0': lambda args: "CANCEL_PRINT",
            'M23': self._prepare_M23,
            'M24': lambda args: "RESUME",
            'M25': lambda args: "PAUSE",
            'M32': self._prepare_M32,
            'M98': self._prepare_M98,
            'M120': lambda args: "SAVE_GCODE_STATE STATE=PANELDUE",
            'M121': lambda args: "RESTORE_GCODE_STATE STATE=PANELDUE",
            'M999': lambda args: "FIRMWARE_RESTART"
        }

        # Register Gcodes
        self.gcode.register_command("M290", self.cmd_M290)
        self.gcode.register_command("PANELDUE_M32", self.cmd_PANELDUE_M32)
        self.gcode.register_command("PANELDUE_M98", self.cmd_PANELDUE_M98)
        self.gcode.register_command("PANELDUE_BEEP", self.cmd_PANELDUE_BEEP)

    def handle_ready(self):
        # Get printer objects
        status_objs = [
            'toolhead', 'virtual_sdcard', 'pause_resume',
            'heater_bed', 'extruder', 'fan', 'display_status']
        for name in status_objs:
            obj = self.printer.lookup_object(name, None)
            if obj is not None:
                self.printer_objects[name] = obj

        # Get extruder objects
        self.extruder_count = 0
        if 'extruder' in self.printer_objects:
            self.extruder_count = 1
            while True:
                extruder_name = "extruder%d" % self.extruder_count
                obj = self.printer.lookup_object(extruder_name, None)
                if obj is not None:
                    self.printer_objects[extruder_name] = obj
                else:
                    break
                self.extruder_count += 1
        self.is_ready = True

    def handle_disconnect(self):
        # Tell the PD that we are shutting down
        self.write_response({'status': 'S'})

    def process_line(self, line):
        # If we find M112 in the line then skip verification
        if "M112" in line.upper():
            self.serial_display.emergency_shutdown()
            return

        # Get line number
        line_index = line.find(' ')
        try:
            line_no = int(line[1:line_index])
        except Exception:
            line_index = -1
            line_no = None

        # Verify checksum
        cs_index = line.rfind('*')
        try:
            checksum = int(line[cs_index+1:])
        except Exception:
            # Invalid checksum, do not process
            msg = "!! Invalid Checksum"
            if line_no is not None:
                msg = " Line Number: %d" % line_no
            logging.exception("PanelDue: " + msg)
            raise self.gcode.error(msg)

        # Checksum is calculated by XORing every byte in the line other
        # than the checksum itself
        calculated_cs = 0
        for c in line[:cs_index]:
            calculated_cs ^= ord(c)
        if calculated_cs & 0xFF != checksum:
            msg = "!! Invalid Checksum"
            if line_no is not None:
                msg = " Line Number: %d" % line_no
            logging.info("PanelDue: " + msg)
            raise self.gcode.error(msg)

        self._run_gcode(line[line_index+1:cs_index])

    def _run_gcode(self, script):
        # Execute the gcode.  Check for special RRF gcodes that
        # require special handling
        parts = script.split()
        cmd = parts[0].strip()

        # Check for commands that query state and require immediate response
        if cmd in self.direct_gcodes:
            params = {}
            for p in parts[1:]:
                arg = p[0].lower() if p[0].lower() in "psr" else "p"
                try:
                    val = int(p[1:].strip()) if arg in "sr" else p[1:].strip()
                except Exception:
                    msg = "paneldue: Error parsing direct gcode %s" % (script)
                    self.handle_gcode_response("!! " + msg)
                    logging.exception(msg)
                    return
                params["arg_" + arg] = val
            func = self.direct_gcodes[cmd]
            func(**params)
            return

        # Prepare GCodes that require special handling
        if cmd in self.special_gcodes:
            func = self.special_gcodes[cmd]
            script = func(parts[1:])

        try:
            self.gcode.run_script(script)
        except Exception:
            msg = "Error executing script %s" % script
            self.handle_gcode_response("!! " + msg)
            logging.exception(msg)

    def _clean_filename(self, filename):
        # Remove drive number
        if filename.startswith("0:/"):
            filename = filename[3:]
        # Remove initial "gcodes" folder.  This is necessary
        # due to the HACK in the paneldue_M20 gcode.
        if filename.startswith("gcodes/"):
            filename = filename[6:]
        elif filename.startswith("/gcodes/"):
            filename = filename[7:]
        # Start with a "/" so the gcode parser can correctly
        # handle files that begin with digits or special chars
        if filename[0] != "/":
            filename = "/" + filename
        return filename

    def _prepare_M23(self, args):
        filename = self._clean_filename(args[0].strip())
        return "M23 " + filename

    def _prepare_M32(self, args):
        return "PANELDUE_M32 P=" + args[0].strip()

    def _prepare_M98(self, args):
        return "PANELDUE_M98 P=" + args[0][1:].strip()

    def handle_gcode_response(self, response):
        # Only queue up "non-trivial" gcode responses.  At the
        # moment we'll handle state changes and errors
        if "Klipper state" in response \
                or response.startswith('!!'):
            self.last_gcode_response = response
        else:
            for key in self.non_trivial_keys:
                if key in response:
                    self.last_gcode_response = response
                    return

    def write_response(self, response):
        self.serial_display.send(json.dumps(response) + "\r\n")

    def _get_state(self, printer_status):
        # PanelDue States applicable to Klipper:
        # I = idle, P = printing from SD, S = stopped (shutdown),
        # C = starting up (not ready), A = paused, D = pausing,
        # B = busy
        if self.printer.is_shutdown():
            return 'S'

        is_active = printer_status['virtual_sdcard'].get('is_active', False)
        paused = printer_status['pause_resume'].get('is_paused', False)
        if paused:
            if is_active:
                return 'D'
            else:
                return 'A'

        if is_active:
            return 'P'

        if printer_status['gcode']['busy']:
            return 'B'

        return 'I'

    def _get_printer_status(self):
        eventtime = self.reactor.monotonic()
        printer_status = {
            'gcode': {}, 'toolhead': {}, 'virtual_sdcard': {},
            'pause_resume': {}, 'heater_bed': {}, 'extruder': {},
            'fan': {}, 'display_status': {}}
        for name, obj in self.printer_objects.iteritems():
            printer_status[name] = obj.get_status(eventtime)
        return printer_status

    def _run_paneldue_M408(self, arg_r=None, arg_s=1):
        response = {}
        sequence = arg_r
        response_type = arg_s

        # Send gcode responses
        if sequence is not None and self.last_gcode_response:
            response['seq'] = sequence + 1
            response['resp'] = self.last_gcode_response
            self.last_gcode_response = None

        if response_type == 1:
            # Extended response Request
            response['myName'] = self.machine_name
            response['firmwareName'] = self.firmware_name
            response['numTools'] = self.extruder_count
            response['geometry'] = self.kinematics
            response['axes'] = 3

        if not self.is_ready:
            # Klipper is still starting up, do not query status
            response['status'] = 'C'
            self.write_response(response)
            return

        printer_status = self._get_printer_status()
        state = self._get_state(printer_status)
        response['status'] = state
        response['babystep'] = round(printer_status['gcode']['homing_zpos'], 3)

        # Current position
        pos = printer_status['toolhead']['position']
        response['pos'] = [round(p, 2) for p in pos[:3]]
        homed_pos = printer_status['toolhead']['homed_axes']
        response['homed'] = [int(a in homed_pos) for a in "xyz"]
        sfactor = round(printer_status['gcode']['speed_factor'] * 100, 2)
        response['sfactor'] = sfactor

        # Print Progress Tracking
        sd_status = printer_status['virtual_sdcard']
        fname = sd_status.get('current_file', "")
        if fname:
            # We know a file has been loaded, initialize metadata
            if self.current_file != fname:
                self.current_file = fname
                file_manager = self.printer.lookup_object('file_manager')
                filelist = file_manager.get_file_list()
                self.file_metadata = filelist.get(fname, {})
            progress = printer_status['virtual_sdcard']['progress']
            # progress and print tracking
            if progress:
                response['fraction_printed'] = round(progress, 3)
                est_time = self.file_metadata.get('estimated_time', 0)
                if est_time > MIN_EST_TIME:
                    # file read estimate
                    times_left = [int(est_time - est_time * progress)]
                    # filament estimate
                    est_total_fil = self.file_metadata.get('filament_total')
                    if est_total_fil:
                        cur_filament = sd_status['filament_used']
                        fpct = min(1., cur_filament / est_total_fil)
                        times_left.append(int(est_time - est_time * fpct))
                    # object height estimate
                    obj_height = self.file_metadata.get('object_height')
                    if obj_height:
                        cur_height = printer_status['gcode']['move_zpos']
                        hpct = min(1., cur_height / obj_height)
                        times_left.append(int(est_time - est_time * hpct))
                else:
                    # The estimated time is not in the metadata, however we
                    # can still provide an estimate based on file progress
                    duration = sd_status['print_duration']
                    times_left = [int(duration / progress - duration)]
                response['timesLeft'] = times_left
        else:
            # clear filename and metadata
            self.current_file = ""
            self.file_metadata = {}

        fan_speed = printer_status['fan'].get('speed')
        if fan_speed is not None:
            response['fanPercent'] = [round(fan_speed * 100, 1)]

        if self.extruder_count > 0:
            extruder_name = printer_status['toolhead']['extruder']
            tool = 0
            if extruder_name != "extruder":
                tool = int(extruder_name[-1])
            response['tool'] = tool

        # Report Heater Status
        efactor = round(printer_status['gcode']['extrude_factor'] * 100., 2)
        heaters = ['heater_bed']
        for e in range(self.extruder_count):
            name = "extruder"
            if e:
                name = "extruder%d" % e
            heaters.append(name)

        for name in heaters:
            temp = round(printer_status[name].get('temperature', 0.0), 1)
            target = round(printer_status[name].get('target', 0.0), 1)
            response.setdefault('heaters', []).append(temp)
            response.setdefault('active', []).append(target)
            response.setdefault('standby', []).append(target)
            response.setdefault('hstat', []).append(2 if target else 0)
            if name.startswith('extruder'):
                response.setdefault('efactor', []).append(efactor)
                response.setdefault('extr', []).append(round(pos.e, 2))

        # Display message (via M117)
        msg = printer_status['display_status'].get('message')
        if msg and msg != self.last_message:
            response['message'] = msg
            # reset the message so it only shows once.  The paneldue
            # is strange about this, and displays it as a full screen
            # notification
        self.last_message = msg

        self.write_response(response)

    def _run_paneldue_M20(self, arg_p, arg_s=0):
        response_type = arg_s
        if response_type != 2:
            logging.info(
                "PanelDue: Cannot process response type %d in M20"
                % (response_type))
            return
        path = arg_p

        # Strip quotes if they exist
        path = path.strip('\"')

        # Path should come in as "0:/macros, or 0:/<gcode_folder>".  With
        # repetier compatibility enabled, the default folder is root,
        # ie. "0:/"
        if path.startswith("0:/"):
            path = path[2:]
        response = {'dir': path}
        response['files'] = []

        if path == "/macros":
            response['files'] = self.available_macros.keys()
        else:
            # HACK: The PanelDue has a bug where it does not correctly detect
            # subdirectories if we return the root as "/".  Thus we need to
            # return it as "/gcodes", then assume that the "/gcodes" directory
            # is root whenever we receive a file request from the PanelDue
            if path == "/":
                response['dir'] = "/gcodes"
                path = ""
            elif path.startswith("/gcodes"):
                path = path[8:]
            file_manager = self.printer.lookup_object('file_manager', None)
            if file_manager is not None:
                masterlist = file_manager.get_file_list()
                filelist = []
                for fname in masterlist:
                    if fname.startswith(path):
                        fname = fname[len(path):]
                        if fname[0] == '/':
                            fname = fname[1:]
                        parts = fname.split('/')
                        if len(parts) == 1:
                            filelist.append(parts[0])
                        elif parts:
                            dir_name = "*" + parts[0]
                            if dir_name not in filelist:
                                filelist.append(dir_name)
                if filelist:
                    response['files'] = filelist
        self.write_response(response)

    def _run_paneldue_M36(self, arg_p=None):
        response = {}
        filename = arg_p
        v_sd = self.printer_objects.get('virtual_sdcard')
        if filename is None:
            # PanelDue is requesting file information on a
            # currently printed file
            active = False
            if v_sd is not None:
                curtime = self.reactor.monotonic()
                sd_status = v_sd.get_status(curtime)
                filename = sd_status['current_file']
                active = sd_status['is_active']
            if not filename or not active:
                # Either no file printing or no virtual_sdcard
                response['err'] = 1
                self.write_response(response)
                return
            else:
                response['fileName'] = filename.split("/")[-1]

        if filename[0] == '/':
            filename = filename[1:]
        # Remove "gcodes/" due to the HACK in M20
        if filename.startswith('gcodes/'):
            filename = filename[7:]

        filedata = None
        file_manager = self.printer.lookup_object('file_manager', None)
        if file_manager is not None:
            masterlist = file_manager.get_file_list()
            filedata = masterlist.get(filename)
        if filedata is not None:
            response['err'] = 0
            response['size'] = filedata['size']
            # workaround for PanelDue replacing the first "T" found
            response['lastModified'] = "T" + filedata['modified']
            slicer = filedata.get('slicer')
            if slicer is not None:
                response['generatedBy'] = slicer
            height = filedata.get('object_height')
            if height is not None:
                response['height'] = round(height, 2)
            layer_height = filedata.get('layer_height')
            if layer_height is not None:
                response['layerHeight'] = round(layer_height, 2)
            filament = filedata.get('filament_total')
            if filament is not None:
                response['filament'] = [round(filament, 1)]
            est_time = filedata.get('estimated_time')
            if est_time is not None:
                response['printTime'] = int(est_time + .5)
        else:
            response['err'] = 1
        self.write_response(response)

    def cmd_PANELDUE_M32(self, gcmd):
        # Start Print
        filename = self._clean_filename(gcmd.get("P").strip())
        start_cmd = "M23 " + filename + "\n" + "M24"
        self.gcode.run_script_from_command(start_cmd)

    def cmd_PANELDUE_M98(self, gcmd):
        macro = gcmd.get('P', gcmd)
        name_start = macro.rfind('/') + 1
        macro = macro[name_start:]
        cmd = self.available_macros.get(macro)
        if cmd is not None:
            self.gcode.run_script_from_command(cmd)
        else:
            gcmd.respond_info("Macro %s invalid" % macro)

    def cmd_PANELDUE_BEEP(self, gcmd):
        freq = gcmd.get_int("FREQUENCY")
        duration = gcmd.get_float("DURATION")
        duration = int(duration * 1000.)
        self.write_response({'beep_freq': freq, 'beep_length': duration})

    def cmd_M290(self, gcmd):
        # apply gcode offset (relative)
        offset = gcmd.get_float('Z')
        self.gcode.run_script_from_command(
            "SET_GCODE_OFFSET Z_ADJUST=%.2f MOVE=1" % offset)


# BTT TFT Constants
BTT_RESET_METHODS = {
    'dtr': 'DTR', 'mcu': 'MCU_GPIO', 'pi': "PI_GPIO",
    'default': None}
BTT_RESET_SCRIPT_NAME = "btttft_pi_gpio_reset.sh"
BTT_RESET_SCRIPT = \
    """
    #!/bin/sh
    echo "{0}" > /sys/class/gpio/export
    echo "out" > /sys/class/gpio/gpio{0}/direction
    echo "0" > /sys/class/gpio/gpio{0}/value
    sleep .1s
    echo "1" > /sys/class/gpio/gpio{0}/value
    echo "{0}" > /sys/class/gpio/unexport
    """

class BTTDisplay:
    def __init__(self, config, display):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.gcode = self.printer.lookup_object('gcode')
        self.name = "BigTreeTech TFT"
        self.serial_display = display
        self.mutex = self.reactor.mutex()
        self.is_ready = False

        # printer status reporting timer
        self.m27_refresh_time = 0
        self.m105_refresh_time = 0
        self.refresh_count = 0
        reactor = self.printer.get_reactor()
        self.status_timer = reactor.register_timer(
            self._handle_status_update)

        # set up the reset method and pins
        self.script_path = ""
        self.reset_pin = self.reset_cmd = None
        self.reset_method = config.getchoice(
            'reset_method', BTT_RESET_METHODS, 'default')
        if self.reset_method == 'MCU_GPIO':
            ppins = self.printer.lookup_object('pins')
            self.reset_pin = ppins.setup_pin(
                'digital_out', config.get('reset_pin'))
            self.reset_pin.setup_max_duration(0.)
            self.reset_pin.setup_start_value(1., 1.)
        elif self.reset_method == "PI_GPIO":
            self.script_path = os.path.join(
                tempfile.gettempdir(), BTT_RESET_SCRIPT_NAME)
            self.reset_pin = config.getint('reset_pin')
            if self.reset_pin in [14, 15]:
                raise config.error(
                    "serial_display: Cannot use pin %d as the reset_pin, it "
                    "is reserved for the uart" % (self.reset_pin))
            shell_command = self.printer.load_object(config, 'shell_command')
            self.reset_cmd = shell_command.load_shell_command(
                "sudo " + self.script_path)
        # register printer event handlers
        self.printer.register_event_handler(
            "klippy:ready", self._handle_ready)

        self.btt_gcodes = [
            'M20', 'M33', 'M27', 'M105', 'M114', 'M115',
            'M155', 'M220', 'M221']
        self.ignored_gcodes = [
            'M500', 'M503', 'M92', 'M851', 'M420', 'M81', 'M150',
            'M48', 'M280', 'M420']
        self.need_ack = False

        # register gcodes
        for gc in self.btt_gcodes:
            name = "BTT_" + gc
            func = getattr(self, "cmd_" + name)
            self.gcode.register_command(name, func)
        self.gcode.register_command("M290", self.cmd_M290)

        # XXX - The following gcodes are currently ignored, but can likely be
        #       implemented:
        # M150 - Neopixel?
        # M48 - PROBE ACCURACY
        # M280 - Servo position (bltouch?)

    def _handle_ready(self):
        if self.serial_display.connected:
            self.reset_device()
            waketime = self.reactor.monotonic() + .1
            self.reactor.update_timer(self.status_timer, waketime)
            self.is_ready = True

    def reset_device(self):
        if self.reset_method == "MCU_GPIO":
            # attempt to reset the device
            toolhead = self.printer.lookup_object('toolhead')
            print_time = toolhead.get_last_move_time()
            self.reset_pin.set_digital(print_time, 0)
            self.reset_pin.set_digital(print_time + .1, 1)
            toolhead.wait_moves()
            self.serial_display.ser.reset_input_buffer()
        elif self.reset_method == "PI_GPIO":
            script = BTT_RESET_SCRIPT.format(self.reset_pin)
            try:
                with open(self.script_path, 'w') as f:
                    f.write(script)
                os.chmod(self.script_path, 0o774)
                self.reset_cmd.run(verbose=False)
                self.serial_display.ser.reset_input_buffer()
                os.remove(self.script_path,)
            except Exception:
                logging.exception(
                    "serial_display: error resetting BTT-TFT device "
                    "via the Pi's GPIO")
        elif self.reset_method == "DTR":
            # attempt to reset the device by toggling DTR
            self.serial_display.ser.dtr = True
            eventtime = self.reactor.monotonic()
            self.reactor.pause(eventtime + .1)
            self.serial_display.ser.reset_input_buffer()
            self.serial_display.ser.dtr = False

    def handle_disconnect(self):
        if self.need_ack:
            self.serial_display.send("ok\n")
            self.need_ack = False
        self.reactor.update_timer(self.status_timer, self.reactor.NEVER)

    def _handle_status_update(self, eventtime):
        if self.refresh_count % 2:
            # report fan status
            fan = self.printer.lookup_object('fan', None)
            if fan is not None:
                fsts = fan.get_status(eventtime)
                speed = int(fsts['speed'] * 255 + .5)
                self.serial_display.send("echo: F0:%s\n" % speed)

        if self.m27_refresh_time and \
                not self.refresh_count % self.m27_refresh_time:
            heaters = self.printer.lookup_object('heaters')
            # XXX - need to make the below method public so I can call it
            self.serial_display.send(heaters._get_temp(eventtime) + "\n")

        if self.m105_refresh_time and \
                not self.refresh_count % self.m105_refresh_time:
            vsd = self.printer.lookup_object('virtual_sdcard', None)
            if vsd is not None:
                # XXX - add a method to the virtual sdcard that gets
                # this string rather than parsing it ourselves
                sd_status = vsd.get_status(eventtime)
                if sd_status['current_file']:
                    pos = sd_status['file_position']
                    size = vsd.file_size
                    self.serial_display.send(
                        "SD printing byte %d/%d\n" % (pos, size))
                else:
                    self.serial_display.send("Not SD printing.\n")

        self.refresh_count += 1
        return eventtime + 1.

    def process_line(self, line):
        if not self.is_ready:
            return
        # The btttft does not send line numbers or checksums
        if "M112" in line.upper():
            self.serial_display.emergency_shutdown()
            self.serial_display.send("ok\n")
            return
        elif "M524" in line.upper():
            # Cancel a print.  The best way to do it in Klipper
            # is emergency shutdown followed by a firmware restart
            # XXX - Like with the paneldue, I may need to execute
            # a delayed restart
            self.serial_display.emergency_shutdown()
            line = "FIRMWARE_RESTART"

        with self.mutex:
            self._process_command(line)

    def _process_command(self, script):
        parts = script.split()
        cmd = parts[0].upper()
        # Just send back "ok" for these gcodes
        if cmd in self.ignored_gcodes:
            self.serial_display.send("ok\n")
            return
        elif cmd in self.btt_gcodes:
            # create the extended version
            new_cmd = "BTT_" + cmd
            if cmd == "M33":
                # handle file names
                new_cmd += " P=" + parts[1].strip()
            else:
                for part in parts[1:]:
                    param = part[0].upper()
                    if param in "PSR":
                        new_cmd += " " + param + "=" + part[1:].strip()
                    else:
                        new_cmd += " P=" + part.strip()
            script = new_cmd

        self.need_ack = True
        try:
            self.gcode.run_script(script)
        except Exception:
            # XXX - return error?
            msg = "BTT-TFT: Error executing script %s" % (script)
            logging.exception(msg)

        # ack if not already done
        if self.need_ack:
            self.serial_display.send("ok\n")

    def handle_gcode_response(self, response):
        if not self.is_ready:
            return
        lines = response.split("\n")
        for line in lines:
            start = line[:2]
            if start == "ok":
                continue
            elif start == "//":
                # XXX - we may want to do this like the paneldue
                # and only show certain items
                line = "echo:" + line[2:]
            elif start == "!!":
                line = "Error:" + line[2:]
            self.serial_display.send(response + "\n")

    def cmd_BTT_M115(self, gcmd):
        version = self.printer.get_start_args().get('software_version')
        kw = {"FIRMWARE_NAME": "Klipper", "FIRMWARE_VERSION": version}
        msg = " ".join(["%s:%s" % (k, v) for k, v in kw.items()]) + "\n"
        vsd = self.printer.lookup_object('virtual_sdcard', None)
        has_vsd = int(vsd is not None)
        # Add Marlin style "capabilities"
        capabilities = {
            'EEPROM': 0, 'AUTOREPORT_TEMP': 1, 'AUTOLEVEL': 0, 'Z_PROBE': 0,
            'LEVELING_DATA': 0, 'SOFTWARE_POWER': 0, 'TOGGLE_LIGHTS': 0,
            'CASE_LIGHT_BRIGHTNESS': 0, 'EMERGENCY_PARSER': 1,
            'SDCARD': has_vsd, 'AUTO_REPORT_SD_STATUS': has_vsd}

        msg += "\n".join(["Cap:%s:%d" % (c, v) for c, v
                          in capabilities.iteritems()])
        self.serial_display.send(msg + "\n")

    def cmd_BTT_M20(self, gcmd):
        vsd = self.printer.lookup_object('virtual_sdcard', None)
        files = []
        if vsd is not None:
            files = vsd.get_file_list()
        self.serial_display.send("Begin file list\n")
        for fname, fsize in files:
            if "/" in fname:
                fname = "/" + fname
            self.serial_display.send("%s %d\n" % (fname, fsize))
        self.serial_display.send("End file list\n")

    def cmd_BTT_M27(self, gcmd):
        interval = gcmd.get_int('S', None)
        if interval is not None:
            self.m27_refresh_time = interval
            return
        eventtime = self.printer.get_reactor().monotonic()
        vsd = self.printer.lookup_object('virtual_sdcard', None)
        if vsd is not None:
            # XXX - add a method to the virtual sdcard that gets
            # this string rather than parsing it ourselves
            sd_status = vsd.get_status(eventtime)
            if sd_status['current_file']:
                pos = sd_status['file_position']
                size = vsd.file_size
                msg = "ok SD printing byte %d/%d\n" % (pos, size)
            else:
                msg = "ok Not SD printing.\n"
            self.need_ack = False
            self.serial_display.send(msg)

    def cmd_BTT_M33(self, gcmd):
        fname = gcmd.get('P')
        if fname[0] != "/":
            fname = "/" + fname
        self.serial_display.send("%s\n" % fname)

    def cmd_BTT_M105(self, gcmd):
        eventtime = self.reactor.monotonic()
        heaters = self.printer.lookup_object('heaters')
        # XXX - need to make the below method public so I can call it
        msg = "ok " + heaters._get_temp(eventtime) + "\n"
        self.need_ack = False
        self.serial_display.send(msg)

    def cmd_BTT_M114(self, gcmd):
        eventtime = self.reactor.monotonic()
        p = self.gcode.get_status(eventtime)['gcode_position']
        self.serial_display.send("X:%.3f Y:%.3f Z:%.3f E:%.3f\n" % tuple(p))

    def cmd_BTT_M220(self, gcmd):
        s_factor = gcmd.get_float('S', None)
        if s_factor is None:
            eventtime = self.reactor.monotonic()
            gcs = self.gcode.get_status(eventtime)
            feed = int(gcs['speed_factor'] * 100 + .5)
            self.serial_display.send("echo: FR:%d%%\n" % (feed))
        else:
            self.gcode.run_script_from_command("M220 S%f" % (s_factor))

    def cmd_BTT_M221(self, gcmd):
        e_factor = gcmd.get_float('S', None)
        if e_factor is None:
            eventtime = self.reactor.monotonic()
            gcs = self.gcode.get_status(eventtime)
            flow = int(gcs['extrude_factor'] * 100 + .5)
            self.serial_display.send("echo: E0 Flow: %d%%\n" % (flow))
        else:
            self.gcode.run_script_from_command("M221 S%f" % (e_factor))

    def cmd_BTT_M155(self, gcmd):
        # set up temperature autoreporting.  Note that it
        # doesn't appear that the TFT currently implements this,
        # however the code is set to do so in the future.  Well
        # go ahead and prepare for it now
        interval = gcmd.get_int('S')
        self.m105_refresh_time = interval

    def cmd_M290(self, gcmd):
        # apply gcode offset (relative)
        offset = gcmd.get_float('Z')
        self.gcode.run_script_from_command(
            "SET_GCODE_OFFSET Z_ADJUST=%.2f MOVE=1" % offset)

def load_config(config):
    return SerialDisplay(config)
