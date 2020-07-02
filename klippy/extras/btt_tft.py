# BigTreeTech TFT LCD display support
#
# Copyright (C) 2020  Eric Callahan <arksine.code@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import serial
import os
import time
import json
import errno
import logging
import tempfile
import util

class SerialConnection:
    def __init__(self, config, btt_display):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.btt_display = btt_display
        self.port = config.get('serial')
        self.baud = config.get('baud', 115200)
        self.ser = self.fd = None
        self.connected = False
        self.fd_handle = None
        self.partial_input = ""
        self.is_busy = False
        self.pending_commands = []

    def disconnect(self):
        if self.connected:
            if self.fd_handle is not None:
                self.reactor.unregister_fd(self.fd_handle)
                self.fd_handle = None
            self.connected = False
            self.ser.close()
            self.ser = None

    def connect(self):
        start_time = connect_time = self.reactor.monotonic()
        while not self.connected:
            if connect_time > start_time + 30.:
                logging.info("btt_tft: Unable to connect, aborting")
                break
            try:
                # XXX - sometimes the port cannot be exclusively locked, this
                # would likely be due to a restart where the serial port was
                # not correctly closed.  Maybe don't use exclusive mode?
                self.ser = serial.Serial(
                    self.port, self.baud, timeout=0, exclusive=True)
            except (OSError, IOError, serial.SerialException) as e:
                logging.warn("btt_tft: unable to open port: %s", e)
                connect_time = self.reactor.pause(connect_time + 2.)
                continue
            self.fd = self.ser.fileno()
            util.set_nonblock(self.fd)
            self.fd_handle = self.reactor.register_fd(
                self.fd, self._process_data)
            self.connected = True
            logging.info(
                "btt_tft: BigTreeTech TFT Serial Connection established")

    def get_serial(self):
        return self.ser

    def _process_data(self, eventtime):
        # Process incoming data using same method as gcode.py
        try:
            data = os.read(self.fd, 4096)
        except os.error:
            if self.printer.is_shutdown():
                logging.exception("btt_tft: read error while shutdown")
                self.disconnect()
            return

        if not data:
            # possibly an error, disconnect
            self.disconnect()
            logging.info("btt_tft: No data received, disconnecting")
            return

        # Remove null bytes, separate into lines
        data = data.strip('\x00')
        lines = data.split('\n')
        lines[0] = self.partial_input + lines[0]
        self.partial_input = lines.pop()
        pending_commands = self.pending_commands
        pending_commands.extend(lines)
        if self.is_busy or len(pending_commands) > 1:
            if len(pending_commands) < 20:
                # Check for M112 out-of-order
                for line in lines:
                    if "M112" in line.upper():
                        gcode = self.printer.lookup_object('gcode')
                        gcode.cmd_M112(None)
            if self.is_busy:
                if len(pending_commands) > 20:
                    # Stop reading input
                    self.reactor.unregister_fd(self.fd_handle)
                    self.fd_handle = None
                return
        self.is_busy = True
        while pending_commands:
            self.pending_commands = []
            self._process_commands(pending_commands)
            pending_commands = self.pending_commands
        self.is_busy = False
        if self.fd_handle is None:
            self.fd_handle = self.reactor.register_fd(
                self.fd, self._process_data)

    def _process_commands(self, commands):
        for cmd in commands:
            cmd = cmd.strip()
            try:
                self.btt_display.process_line(cmd)
            except Exception:
                logging.exception(
                    "btt_tft: GCode Processing Error: " + cmd)
                self.btt_display.handle_gcode_response(
                    "!! GCode Processing Error: " + cmd)

    def send(self, data):
        if self.connected:
            retries = 10
            while data:
                try:
                    sent = os.write(self.fd, data)
                except os.error as e:
                    if e.errno == errno.EBADF or e.errno == errno.EPIPE \
                            or not retries:
                        sent = 0
                    else:
                        retries -= 1
                        continue
                if sent:
                    data = data[sent:]
                else:
                    logging.exception(
                        "btt_tft: Error writing data,"
                        " closing serial connection")
                    self.disconnect()
                    return

class BTTDisplay:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.gcode = self.printer.lookup_object('gcode')
        self.connection = SerialConnection(config, self)
        self.mutex = self.reactor.mutex()
        self.is_ready = False

        # printer status reporting timer
        self.m27_refresh_time = 0
        self.m105_refresh_time = 0
        self.refresh_count = 0
        reactor = self.printer.get_reactor()
        self.status_timer = reactor.register_timer(
            self._do_status_update)

        # set up the reset method and pins
        self.script_path = ""
        self.reset_pin = self.reset_cmd = None
        reset_methods = {'mcu': 'mcu', 'dtr': 'dtr', 'none': None}
        self.reset_method = config.getchoice(
            'reset_method', reset_methods, 'none')
        if self.reset_method == 'mcu':
            ppins = self.printer.lookup_object('pins')
            self.reset_pin = ppins.setup_pin(
                'digital_out', config.get('reset_pin'))
            self.reset_pin.setup_max_duration(0.)
            self.reset_pin.setup_start_value(1., 1.)

        # register printer event handlers
        self.printer.register_event_handler(
            "klippy:ready", self._handle_ready)
        self.printer.register_event_handler(
            "klippy:connect", self.connection.connect)
        self.printer.register_event_handler(
            "klippy:disconnect", self._handle_disconnect)
        self.printer.register_event_handler(
            "gcode:respond", self._handle_gcode_response)

        # load display status so we can get M117 messages
        self.printer.load_object(config, 'display_status')

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
        if self.connection.connected:
            self.reset_device()
            waketime = self.reactor.monotonic() + .1
            self.reactor.update_timer(self.status_timer, waketime)
            self.is_ready = True

    def reset_device(self):
        ser = self.connection.get_serial()
        if ser is None:
            logging.info("btt_tft: Serial Connection Failed, cannot reset")
            return
        if self.reset_method == "mcu":
            # attempt to reset the device
            toolhead = self.printer.lookup_object('toolhead')
            print_time = toolhead.get_last_move_time()
            self.reset_pin.set_digital(print_time, 0)
            self.reset_pin.set_digital(print_time + .1, 1)
            toolhead.wait_moves()
            ser.reset_input_buffer()
        elif self.reset_method == "dtr":
            # attempt to reset the device by toggling DTR
            ser.dtr = True
            eventtime = self.reactor.monotonic()
            self.reactor.pause(eventtime + .1)
            ser.reset_input_buffer()
            ser.dtr = False

    def _handle_disconnect(self):
        if self.need_ack:
            self.connection.send("ok\n")
            self.need_ack = False
        self.reactor.update_timer(self.status_timer, self.reactor.NEVER)
        self.connection.disconnect()

    def _handle_gcode_response(self, response):
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
            self.connection.send(response + "\n")

    def _do_status_update(self, eventtime):
        if self.refresh_count % 2:
            # report fan status
            fan = self.printer.lookup_object('fan', None)
            if fan is not None:
                fsts = fan.get_status(eventtime)
                speed = int(fsts['speed'] * 255 + .5)
                self.connection.send("echo: F0:%s\n" % speed)

        if self.m27_refresh_time and \
                not self.refresh_count % self.m27_refresh_time:
            heaters = self.printer.lookup_object('heaters')
            # XXX - need to make the below method public so I can call it
            self.connection.send(heaters._get_temp(eventtime) + "\n")

        if self.m105_refresh_time and \
                not self.refresh_count % self.m105_refresh_time:
            vsd = self.printer.lookup_object('virtual_sdcard', None)
            if vsd is not None:
                # XXX - add a method to the virtual sdcard that gets
                # this string rather than accessing the vsd's attributes
                pos = vsd.file_position
                if pos:
                    size = vsd.file_size
                    self.connection.send(
                        "SD printing byte %d/%d\n" % (pos, size))
                else:
                    self.connection.send("Not SD printing.\n")

        self.refresh_count += 1
        return eventtime + 1.

    def process_line(self, line):
        if not self.is_ready:
            return
        # The btttft does not send line numbers or checksums
        if "M112" in line.upper():
            self.gcode.cmd_M112(None)
            self.connection.send("ok\n")
            return
        elif "M524" in line.upper():
            # Cancel a print.  The best way to do it in Klipper
            # is emergency shutdown followed by a firmware restart
            # XXX - I may need to execute a delayed restart
            self.gcode.cmd_M112(None)
            line = "FIRMWARE_RESTART"

        with self.mutex:
            self._process_command(line)

    def _process_command(self, script):
        parts = script.split()
        cmd = parts[0].upper()
        # Just send back "ok" for these gcodes
        if cmd in self.ignored_gcodes:
            self.connection.send("ok\n")
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
            self.connection.send("ok\n")

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
                          in capabilities.items()])
        self.connection.send(msg + "\n")

    def cmd_BTT_M20(self, gcmd):
        vsd = self.printer.lookup_object('virtual_sdcard', None)
        files = []
        if vsd is not None:
            files = vsd.get_file_list()
        self.connection.send("Begin file list\n")
        for fname, fsize in files:
            if "/" in fname:
                fname = "/" + fname
            self.connection.send("%s %d\n" % (fname, fsize))
        self.connection.send("End file list\n")

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
            pos = vsd.file_position
            if pos:
                size = vsd.file_size
                msg = "ok SD printing byte %d/%d\n" % (pos, size)
            else:
                msg = "ok Not SD printing.\n"
            self.need_ack = False
            self.connection.send(msg)

    def cmd_BTT_M33(self, gcmd):
        fname = gcmd.get('P')
        if fname[0] != "/":
            fname = "/" + fname
        self.connection.send("%s\n" % fname)

    def cmd_BTT_M105(self, gcmd):
        eventtime = self.reactor.monotonic()
        heaters = self.printer.lookup_object('heaters')
        # XXX - need to make the below method public so I can call it
        msg = "ok " + heaters._get_temp(eventtime) + "\n"
        self.need_ack = False
        self.connection.send(msg)

    def cmd_BTT_M114(self, gcmd):
        eventtime = self.reactor.monotonic()
        p = self.gcode.get_status(eventtime)['gcode_position']
        self.connection.send("X:%.3f Y:%.3f Z:%.3f E:%.3f\n" % tuple(p))

    def cmd_BTT_M220(self, gcmd):
        s_factor = gcmd.get_float('S', None)
        if s_factor is None:
            eventtime = self.reactor.monotonic()
            gcs = self.gcode.get_status(eventtime)
            feed = int(gcs['speed_factor'] * 100 + .5)
            self.connection.send("echo: FR:%d%%\n" % (feed))
        else:
            self.gcode.run_script_from_command("M220 S%f" % (s_factor))

    def cmd_BTT_M221(self, gcmd):
        e_factor = gcmd.get_float('S', None)
        if e_factor is None:
            eventtime = self.reactor.monotonic()
            gcs = self.gcode.get_status(eventtime)
            flow = int(gcs['extrude_factor'] * 100 + .5)
            self.connection.send("echo: E0 Flow: %d%%\n" % (flow))
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
    return BTTDisplay(config)
