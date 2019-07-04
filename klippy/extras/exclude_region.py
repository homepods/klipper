# Exclude moves toward and inside set regions
#
# Copyright (C) 2019  Eric Callahan <arksine.code@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.

import math
import logging

def parse_pair(pair):
    pts = pair.strip().split(',', 1)
    if len(pts) != 2:
        raise Exception
    return [float(p.strip()) for p in pts]

class RectRegion:
    def __init__(self, minpt, maxpt):
        self.xmin = minpt[0]
        self.ymin = minpt[1]
        self.xmax = maxpt[0]
        self.ymax = maxpt[1]
    def check_within(self, pos):
        return (self.xmin <= pos[0] <= self.xmax and
                self.ymin <= pos[1] <= self.ymax)

class CircRegion:
    def __init__(self, center, radius):
        self.x = center[0]
        self.y = center[1]
        self.radius = radius
    def check_within(self, pos):
        a = self.x - pos[0]
        b = self.y - pos[1]
        dist_from_pt = math.sqrt(a*a + b*b)
        return dist_from_pt <= self.radius

class ExcludeRegion:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.gcode = self.printer.lookup_object('gcode')
        # Temporary workaround to get skew_correction to register
        # its "klippy:ready" event handler before Exclude Region.  Exclude
        # Region needs to be the highest priority transform, thus it must be
        # the last module that calls set_move_transform()
        if config.has_section('skew_correction'):
            self.printer.try_load_module(config, 'skew_correction')
        # Now ExcludeRegion can register its own event handler
        self.printer.register_event_handler("klippy:ready",
                                            self._handle_ready)
        self.regions = {}
        self.objects = []
        self.current_object = ""
        self.last_position = [0., 0., 0., 0.]
        self.last_delta = [0., 0., 0., 0.]
        self.gcode.register_command(
            'EXCLUDE_RECT', self.cmd_EXCLUDE_RECT,
            desc=self.cmd_EXCLUDE_RECT_help)
        self.gcode.register_command(
            'EXCLUDE_CIRCLE', self.cmd_EXCLUDE_CIRCLE,
            desc=self.cmd_EXCLUDE_CIRCLE_help)
        self.gcode.register_command(
            'REMOVE_EXCLUDED_REGION', self.cmd_REMOVE_EXCLUDED_REGION,
            desc=self.cmd_REMOVE_EXCLUDED_REGION_help)
        self.gcode.register_command(
            'SET_CURRENT_OBJECT', self.cmd_SET_CURRENT_OBJECT,
            desc=self.cmd_SET_CURRENT_OBJECT_help)
        self.gcode.register_command(
            'CANCEL_OBJECT', self.cmd_CANCEL_OBJECT,
            desc=self.cmd_CANCEL_OBJECT_help)
        self.gcode.register_command(
            'REMOVE_CANCELED_OBJECT', self.cmd_REMOVE_CANCELED_OBJECT,
            desc=self.cmd_REMOVE_CANCELED_OBJECT_help)
        self.gcode.register_command(
            'REMOVE_ALL_EXCLUDED', self.cmd_REMOVE_ALL_EXCLUDED,
            desc=self.cmd_REMOVE_ALL_EXCLUDED_help)
        # debugging
        self.current_region = None
    def _handle_ready(self):
        self.next_transform = self.gcode.set_move_transform(self, force=True)
    def get_position(self):
        self.last_position[:] = self.next_transform.get_position()
        self.last_delta = [0., 0., 0., 0.]
        return list(self.last_position)
    def move(self, newpos, speed):
        if self.current_object in self.objects:
            # Inside cancelled object
            return
        for key, r in self.regions.iteritems():
            if r.check_within(newpos):
                # Inside cancelled region
                if self.current_region is None:
                    self.current_region = key
                    logging.info(
                        "Entered Excluded Region {%s}, coordinate: %.2f, %.2f"
                        % (key, newpos[0], newpos[1]))
                return
        if self.current_region is not None:
            logging.info(
                "Exited Excluded Region {%s}, coordinate: %.2f, %.2f"
                % (self.current_region, newpos[0], newpos[1]))
        self.current_region = None
        self.last_delta = [newpos[i] - self.last_position[i] for i in range(4)]
        self.last_position[:] = newpos
        self.next_transform.move(newpos, speed)
    cmd_EXCLUDE_RECT_help = "Exclude moves to specificed rectanglar region"
    def cmd_EXCLUDE_RECT(self, params):
        name = self.gcode.get_str('NAME', params).upper()
        try:
            minpt = parse_pair(self.gcode.get_str('MIN', params))
            maxpt = parse_pair(self.gcode.get_str('MAX', params))
        except Exception:
            self.gcode.respond_info(
                "exclude_region: Error parsing EXCLUDE_RECT\n%s" %
                (params['#original']))
            return
        self.regions[name] = RectRegion(minpt, maxpt)
    cmd_EXCLUDE_CIRCLE_help = "Exclude moves to specified circular region"
    def cmd_EXCLUDE_CIRCLE(self, params):
        name = self.gcode.get_str('NAME', params).upper()
        try:
            centerpt = parse_pair(self.gcode.get_str('CENTER', params))
        except Exception:
            self.gcode.respond_info(
                "exclude_region: Error parsing EXCLUDE_CIRCLE\n%s" %
                (params['#original']))
            return
        radius = self.gcode.get_float('RADIUS', params)
        self.regions[name] = CircRegion(centerpt, radius)
    cmd_REMOVE_EXCLUDED_REGION_help = "Remove a specified excluded region"
    def cmd_REMOVE_EXCLUDED_REGION(self, params):
        name = self.gcode.get_str('NAME', params).upper()
        if name in self.regions:
            del self.regions[name]
        else:
            self.gcode.respond_info(
                "exclude_region: No region named [%s] to remove" %
                (name))
    cmd_SET_CURRENT_OBJECT_help = "Set the current object as labeled"
    def cmd_SET_CURRENT_OBJECT(self, params):
        self.current_object = self.gcode.get_str('NAME', params).upper()
    cmd_CANCEL_OBJECT_help = "Cancel moves inside a specified objects"
    def cmd_CANCEL_OBJECT(self, params):
        name = self.gcode.get_str('NAME', params).upper()
        if name not in self.objects:
            self.objects.append(name)
    cmd_REMOVE_CANCELED_OBJECT_help = "Remove cancelled object"
    def cmd_REMOVE_CANCELED_OBJECT(self, params):
        name = self.gcode.get_str('NAME', params).upper()
        if name in self.objects:
            self.objects.remove(name)
    cmd_REMOVE_ALL_EXCLUDED_help = "Removes all excluded objects and regions"
    def cmd_REMOVE_ALL_EXCLUDED(self, params):
        self.regions = {}
        self.objects = []

def load_config(config):
    return ExcludeRegion(config)
