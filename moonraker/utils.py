# General Server Utilities
#
# Copyright (C) 2020 Eric Callahan <arksine.code@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license
import logging
import json

DEBUG = True

# Json decodes strings as unicode types in Python 2.x.  This doesn't
# play well with some parts of Klipper (particuarly displays), so we
# need to create an object hook. This solution borrowed from:
#
# https://stackoverflow.com/questions/956867/
#
def byteify(data, ignore_dicts=False):
    if isinstance(data, unicode):
        return data.encode('utf-8')
    if isinstance(data, list):
        return [byteify(i, True) for i in data]
    if isinstance(data, dict) and not ignore_dicts:
        return {byteify(k, True): byteify(v, True)
                for k, v in data.iteritems()}
    return data

def json_loads_byteified(data):
    return byteify(
        json.loads(data, object_hook=byteify), True)

class ServerError(Exception):
    def __init__(self, message, status_code=400):
        Exception.__init__(self, message)
        self.status_code = status_code


class SocketLoggingHandler(logging.Handler):
    def __init__(self, server_manager):
        super(SocketLoggingHandler, self).__init__()
        self.server_manager = server_manager

    def emit(self, record):
        record.msg = "[MOONRAKER]: " + record.msg
        # XXX - Convert log record to dict before sending,
        # the klippy_send function will handle serialization

        self.server_manager.klippy_send(record)
