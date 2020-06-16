# Klippy Status and Subscription Handler
#
# Copyright (C) 2020 Eric Callahan <arksine.code@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license
import re
import logging
from collections import deque

TEMP_STORE_SIZE = 20 * 60
MAX_TICKS = 64

# Status objects require special parsing
def _status_parser(request):
    query_args = request.query_arguments
    args = {}
    for key, vals in query_args.iteritems():
        parsed = []
        for v in vals:
            if v:
                parsed += v.split(',')
        args[key] = parsed
    return args

class StatusHandler:
    def __init__(self, config, notification_cb):
        self.printer = config.get_printer()
        self.send_notification = notification_cb
        self.printer_ready = False
        self.reactor = self.printer.get_reactor()
        self.tick_time = config.getfloat('tick_time', .25, above=0.)
        self.current_tick = 0
        self.temperature_store = {}
        self.available_objects = {}
        self.subscriptions = []
        self.subscription_timer = self.reactor.register_timer(
            self._batch_subscription_handler, self.reactor.NEVER)
        self.poll_ticks = {
            'toolhead': 1,
            'gcode': 1,
            'idle_timeout': 1,
            'pause_resume': 1,
            'fan': 2,
            'virtual_sdcard': 4,
            'extruder.*': 4,
            'heater.*': 4,
            'temperature_fan': 4,
            'gcode_macro.*': 0,
            'default': 16.
        }
        # Fetch user defined update intervals
        for i in range(1, 7):
            modules = config.get('status_tier_%d' % (i), None)
            if modules is None:
                continue
            ticks = 2 ** (i - 1)
            modules = modules.strip().split('\n')
            modules = [m.strip() for m in modules if m.strip()]
            for name in modules:
                if name.startswith("gcode_macro"):
                    # gcode_macros are blacklisted
                    continue
                self.poll_ticks[name] = ticks
        # Override temperature ticks to be 1 second
        self.temperature_ticks = max(1, int(1. / self.tick_time + .5))

        # Register webhooks
        webhooks = self.printer.lookup_object('webhooks')
        webhooks.register_endpoint(
            '/printer/objects', self._handle_object_request)
        webhooks.register_endpoint(
            '/printer/status', self._handle_status_request,
            params={'arg_parser': _status_parser})
        webhooks.register_endpoint(
            '/printer/subscriptions', self._handle_subscription_request,
            ['GET', 'POST'], {'arg_parser': _status_parser})
        webhooks.register_endpoint(
            '/printer/temperature_store', self._handle_temp_store_request)

    def handle_ready(self):
        self.available_objects = {}
        avail_sensors = []
        objs = self.printer.lookup_objects()
        status_objs = {n: o for n, o in objs if hasattr(o, "get_status")}
        eventtime = self.reactor.monotonic()
        for name, obj in status_objs.iteritems():
            attrs = obj.get_status(eventtime)
            self.available_objects[name] = attrs.keys()
            if name == "heaters":
                avail_sensors = attrs['available_sensors']

        # Setup temperature store
        subs = {}
        for sensor in avail_sensors:
            if sensor in self.available_objects:
                self.temperature_store[sensor] = deque(
                    [0]*TEMP_STORE_SIZE, maxlen=TEMP_STORE_SIZE)
                self.poll_ticks[sensor] = self.temperature_ticks
                subs[sensor] = []
        # Subscribe to available sensors now
        self.add_subscripton(subs)
        self.printer_ready = True

    def _batch_subscription_handler(self, eventtime):
        # self.subscriptions is a 2D array, with the inner array
        # arranged in the form of:
        # [<subscripton>, <poll_ticks>]

        # Accumulate ready status subscriptions
        current_subs = {}
        for sub in self.subscriptions:
            # if no remainder then we process this subscription
            if not self.current_tick % sub[1]:
                # no ticks remaining, process
                current_subs.update(sub[0])

        if current_subs:
            status = self._process_status_request(current_subs)
            # Chech the temperature store
            if not self.current_tick % self.temperature_ticks:
                for sensor in self.temperature_store:
                    self.temperature_store[sensor].append(
                        round(status[sensor]['temperature'], 2))
            self.send_notification('status_update', status)

        self.current_tick = (self.current_tick + 1) % MAX_TICKS
        return eventtime + self.tick_time

    def _process_status_request(self, objects):
        if self.printer_ready:
            for name in objects:
                obj = self.printer.lookup_object(name, None)
                if obj is not None and name in self.available_objects:
                    status = obj.get_status(self.reactor.monotonic())
                    # Determine requested attributes.  If empty, return
                    # all requested attributes
                    if not objects[name]:
                        requested_attrs = status.keys()
                    else:
                        requested_attrs = list(objects[name])
                    objects[name] = {}
                    for attr in requested_attrs:
                        val = status.get(attr, "<invalid>")
                        # Don't return callable values
                        if callable(val):
                            continue
                        objects[name][attr] = val
                else:
                    objects[name] = "<invalid>"
        else:
            objects = {"status": "Klippy Not Ready"}
        return objects

    def _handle_object_request(self, web_request):
        web_request.send(dict(self.available_objects))

    def _handle_status_request(self, web_request):
        args = web_request.get_args()
        result = self._process_status_request(args)
        web_request.send(result)

    def _handle_subscription_request(self, web_request):
        method = web_request.get_method()
        if method.upper() == "POST":
            # add a subscription
            args = web_request.get_args()
            if args:
                self.add_subscripton(args)
            else:
                raise web_request.error("Invalid argument")
        else:
            # get subscription info
            result = self.get_sub_info()
            web_request.send(result)

    def _handle_temp_store_request(self, web_request):
        store = {s: list(t) for s, t in self.temperature_store.iteritems()}
        web_request.send(store)

    def stop(self):
        self.printer_ready = False
        self.reactor.update_timer(self.subscription_timer, self.reactor.NEVER)

    def get_poll_ticks(self, obj):
        if obj in self.poll_ticks:
            return self.poll_ticks[obj]
        else:
            for key, poll_ticks in self.poll_ticks.iteritems():
                if re.match(key, obj):
                    return poll_ticks
        return self.poll_ticks['default']

    def get_sub_info(self):
        objects = {}
        poll_times = {}
        for sub in self.subscriptions:
            objects.update(sub[0])
            for key, attrs in sub[0].iteritems():
                poll_times[key] = sub[1] * self.tick_time
                if attrs == []:
                    objects[key] = list(self.available_objects[key])
        return {'objects': objects, 'poll_times': poll_times}

    def get_sub_by_poll_ticks(self, poll_ticks):
        for sub in self.subscriptions:
            if sub[1] == poll_ticks:
                return sub
        return None

    def add_subscripton(self, new_sub):
        if not new_sub:
            return
        for obj in new_sub:
            if obj not in self.available_objects:
                logging.info(
                    "[WEBSERVER] subscription_handler: Subscription Request"
                    " {%s} not available, ignoring" % (obj))
                continue
            poll_ticks = self.get_poll_ticks(obj)
            if poll_ticks == 0:
                # Blacklisted object, cannot subscribe
                continue
            existing_sub = self.get_sub_by_poll_ticks(poll_ticks)
            if existing_sub is not None:
                existing_sub[0][obj] = new_sub[obj]
            else:
                req = {obj: new_sub[obj]}
                self.subscriptions.append([req, poll_ticks])

        waketime = self.reactor.monotonic() + self.tick_time
        self.reactor.update_timer(self.subscription_timer, waketime)
