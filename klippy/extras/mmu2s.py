# Support for the Prusa MMU2S in usb peripheral mode
#
# Copyright (C) 2019  Eric Callahan <arksine.code@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.

import os
import logging
import serial
import threading

MMU2_BAUD = 115200
RESPONSE_TIMEOUT = 45.
MMU_RESP_STATE = {"DISCONNECT": 0, "NACK": 1, "ACK": 2}

class error(Exception):
    pass

class MMU2Serial:
    def __init__(self, config, notifcation_cb):
        self.port = config.get('serial').strip()
        printer = config.get_printer()
        self.reactor = printer.get_reactor()
        self.gcode = printer.lookup_object('gcode')
        self.ser = None
        self.connected = False
        self.mmu_response = None
        self.notifcation_cb = notifcation_cb
        self.partial_response = ""
        self.fd_handle = self.fd = None
    def connect(self, eventtime):
        logging.info("Starting MMU2S connect")
        self._check_bootloader()
        start_time = self.reactor.monotonic()
        while 1:
            connect_time = self.reactor.monotonic()
            if connect_time > start_time + 90.:
                raise error("Unable to connect to MMU2s")
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
    def _check_bootloader(self):
        timeout = 10
        ttyname = os.path.realpath(self.port)
        while timeout:
            for fname in os.listdir('/dev/serial/by-id/'):
                fname = '/dev/serial/by-id/' + fname
                if os.path.realpath(fname) == ttyname and \
                        "Multi_Material" in fname:
                    if "bootloader" in fname:
                        logging.info("mmu2s: Waiting to exit bootloader")
                        break
                    else:
                        return True
                else:
                    logging.info("mmu2s: No device detected")
            self.reactor.pause(1.)
            timeout -= 1
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
            logging.warn("MMU2S disconnected")
            self.connected = False
            self.reactor.unregister_fd(self.fd_handle)
            self.fd_handle = self.fd = None
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
                    logging.info("mmu2s: mmu initiated transfer: %s", line)
                    self.notifcation_cb(line)
            if ack_count > 1:
                logging.warn("mmu2s: multiple acknowledgements recd")
    def send_with_response(self, data, timeout=RESPONSE_TIMEOUT):
        if not self.connected:
            return MMU_RESP_STATE['DISCONNECT'], ""
        self.mmu_response = None
        try:
            self.ser.write(data)
        except serial.SerialException as e:
            logging.warn("MMU2S disconnected")
            self.connected = False
            self.reactor.unregister_fd(self.fd_handle)
            self.fd_handle = self.fd = None
        curtime = self.reactor.monotonic()
        endtime = curtime + timeout
        pause_count = 0
        while self.mmu_response is None:
            if not self.connected:
                return MMU_RESP_STATE['DISCONNECT'], ""
            if curtime > endtime:
                return MMU_RESP_STATE['NACK'], ""
            curtime = self.reactor.pause(curtime + .01)
            pause_count += 1
            if pause_count >= 200:
                self.gcode.respond_info("mmu2s: waiting for response")
                pause_count = 0
        resp = self.mmu_response
        self.mmu_response = None
        return MMU_RESP_STATE['ACK'], resp


class MMU2Commands:
    def __init__(self, serial, gcode):
        self.serial = serial
        self.gcode = gcode
    def _send_command(self, command, retry=True):
        cmd = bytes(command + '\n')
        if 'P' in command:
            status, resp = self.serial.send_with_response(cmd, 3.)
        else:
            status, resp = self.serial.send_with_response(cmd)
        if status == MMU_RESP_STATE['ACK']:
            return resp
        elif status == MMU_RESP_STATE['NACK']:
            # attempt a resend
            if retry:
                response = self._send_command(command, retry=False)
                if response is None:
                    raise self.gcode.error(
                        "mmu2s: no acknowledgment for command %s" %
                        (command))
            else:
                return None
        elif status == MMU_RESP_STATE['DISCONNECT']:
            # Try reconnecting
            self.serial.connect()
            if retry:
                response = self._send_command(command, retry=False)
                if response is None:
                    raise self.gcode.error(
                        "mmu2s: mmu disconnected, cannot send command %s" %
                        (command))
            else:
                return None
    def change_tool(self, tool):
        #  T(n)
        pass
    def load_filament(self, tool):
        # L(n)
        pass
    def set_tmc_mode(self, mode):
        # M(n)
        resp = self._send_command("M%d" % req)
        return resp
    def unload_filament(self, tool):
        # U(n)
        pass
    def reset_mmu(self):
        # X0
        pass
    def read_finda(self):
        # P0 -  only 3 second ack timeout for this command
        pass
    def get_status(self, req):
        # S0 - basic ack
        # S1 - read version
        # S2 - read build number
        # S3 - read drive error
        resp = self._send_command("S%d" % req)
        return resp
    def set_filament_type(self, tool):
        # F(n) - currently does nothing for MMU
        # likely reserved for future use
        pass
    def continue_load(self):
        # C0
        pass
    def eject_filament(self):
        # E(n)
        pass
    def recover(self):
        # R0 - recover after eject
        pass
    def cut_filament(self, tool):
        # K(n)
        pass

class MMU2S:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.gcode = self.printer.lookup_object('gcode')
        self.mmu_serial = MMU2Serial(config, self.mmu_notification)
        self.mmu_commands = MMU2Commands(self.mmu_serial, self.gcode)
        self.current_extruder = 0
        self.gcode.register_command(
            "MMU_GET_STATUS", self.cmd_MMU_GET_STATUS)
        self.gcode.register_command(
            "MMU_SET_TMC", self.cmd_MMU_SET_TMC)
        self.printer.register_event_handler(
            "klippy:connect", self._handle_connect)
        self.printer.register_event_handler(
            "klippy:disconnect", self._handle_disconnect)
        self.printer.register_event_handler(
            "gcode:request_restart", self._handle_restart)
    def _handle_connect(self):
        reactor = self.printer.get_reactor()
        self.mmu_serial.connect(reactor.monotonic())
    def _handle_restart(self, print_time):
        self.mmu_serial.disconnect()
    def _handle_disconnect(self):
        self.mmu_serial.disconnect()
    def mmu_notification(self, data):
        # XXX - Will need to parse these notifications and do something with them
        self.gcode.respond_info("mmu2s: Notification received\n %s", data)
    def cmd_MMU_GET_STATUS(self, params):
        ack = self.mmu_commands.get_status(0)
        version = self.mmu_commands.get_status(1)[:-2]
        build = self.mmu_commands.get_status(2)[:-2]
        errors = self.mmu_commands.get_status(3)[:-2]
        status = ("MMU Status:\nAcknowledge Test: %s\nVersion: %s\n" +
                  "Build Number: %s\nDrive Errors:%s\n")
        self.gcode.respond_info(status % (ack, version, build, errors))
    def cmd_MMU_SET_TMC(self, params):
        mode = self.gcode.get_into('MODE', params)
        self.mmu_commands.set_tmc_mode(mode)

def load_config(config):
    return MMU2S(config)
