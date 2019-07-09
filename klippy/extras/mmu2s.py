# Support for the Prusa MMU2S in usb peripheral mode
#
# Copyright (C) 2019  Eric Callahan <arksine.code@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.

import serial
import threading

MM2_BAUD = 115200

class MMU2Serial:
    def __init__(self, config):
        self.port = config.get('serial').strip()
        self.ser = None
        self.connected = False
        self.mmu_response = ""
        self.fd_handle = self.fd = None
    def _connect(self):
        # XXX - The big question, do we need to receive on the reactor thread?
        # Probably not, but would it hurt?
        pass
    def _disconnect(self):
        pass
    def _handle_mmu_response(self, eventtime):
        pass
    def send(self, data):
        pass

class MMU2S:
    def __init__(self, config):
        self.printer = config.get_printer()
        pass

def load_config(config):
    return MMU2S(config)