# Probe Temperature Compensation Support
#
# Copyright (C) 2019  Eric Callahan <arksine.code@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import probe
import homing
import logging

KELVIN_TO_CELCIUS = -273.15

# Linear interpolation between two values
def lerp(t, v0, v1):
    return (1. - t) * v0 + t * v1

class TemperatureProbe(probe.PrinterProbe):
    def _probe(self, speed):
        self.mcu_probe.set_probing(True)
        return probe.PrinterProbe._probe(self, speed)
    def get_offsets(self):
        z_offset = self.z_offset + self.mcu_probe.homed_offset
        return self.x_offset, self.y_offset, z_offset

class TemperatureProbeEndstopWrapper(probe.ProbeEndstopWrapper):
    def __init__(self, config):
        probe.ProbeEndstopWrapper.__init__(self, config)
        self.gcode = self.printer.lookup_object('gcode')
        self.printer_ready = False
        self.is_probing = False
        self.sensor_temp = 0.
        self.target = 0.
        self.homed_offset = 0.
        offsets = config.get('temp_offsets').split('\n')
        try:
            offsets = [line.split(',', 1) for line in offsets if line.strip()]
            self.probe_offsets = [(float(p[0].strip()), float(p[1].strip()))
                                  for p in offsets]
        except:
            raise config.error(
                "temperature_probe: error parsing probe offsets")
        if len(self.probe_offsets) < 2:
            raise config.error(
                "temperature_probe: need at least 2 temp/offset pairs")
        logging.info("Probe offsets generated:")
        for temp, offset in self.probe_offsets:
            logging.info("(%.2fC,%.4f)" % (temp, offset))
        sensor_type = config.get('sensor_type')
        for s in ["thermistor ", "adc_temperature "]:
            if config.has_section(s + sensor_type):
                self.printer.try_load_module(config, s + sensor_type)
                break
        self.sensor = self.printer.lookup_object('heater').setup_sensor(config)
        mintemp = config.getfloat('min_temp', minval=KELVIN_TO_CELCIUS)
        maxtemp = config.getfloat('max_temp', above=mintemp)
        self.sensor.setup_minmax(mintemp, maxtemp)
        self.sensor.setup_callback(self._temperature_callback)
        self.printer.lookup_object('heater').register_sensor(config, self)
        self.printer.register_event_handler(
            "klippy:ready", self._handle_ready)
        self.printer.register_event_handler(
            "klippy:shutdown", self._handle_shutdown)
        self.gcode.register_command(
            'PROBE_WAIT', self.cmd_PROBE_WAIT,
            desc=self.cmd_PROBE_WAIT_help)
    def _handle_ready(self):
        self.printer_ready = True
    def _handle_shutdown(self):
        self.printer_ready = False
    def _temperature_callback(self, readtime, temp):
        self.sensor_temp = temp
    def get_temperature_offset(self):
        last_idx = len(self.probe_offsets) - 1
        if self.sensor_temp <= self.probe_offsets[0][0]:
            # Clamp offsets below minimum temperature to 0
            return 0.
        elif self.sensor_temp >= self.probe_offsets[last_idx][0]:
            # Clamp offset above max temperature to last offset
            # in table
            return self.probe_offsets[last_idx][1]
        else:
            # Interpolate between points, not over the entire curve,
            # because the change is not linear across all temperatures
            for index in range(last_idx):
                if self.sensor_temp > self.probe_offsets[index][0] and \
                        self.sensor_temp <= self.probe_offsets[index+1][0]:
                    temp_delta = self.probe_offsets[index+1][0] \
                        - self.probe_offsets[index][0]
                    t = (self.sensor_temp - self.probe_offsets[index][0]) \
                        / (temp_delta)
                    offset = lerp(
                        t, self.probe_offsets[index][1],
                        self.probe_offsets[index+1][1])
                    logging.info(
                        "temperature_probe: Temp: %.2f Calculated Offset: %.4f"
                        % (self.sensor_temp, offset))
                    return offset
    def get_temp(self, eventtime):
        return self.sensor_temp, self.target
    def set_probing(self, state):
        self.is_probing = state
    def home_finalize(self):
        toolhead = self.printer.lookup_object('toolhead')
        cur_pos = toolhead.get_position()
        offset = self.get_temperature_offset()
        if self.is_probing:
            offset_diff = offset - self.homed_offset
            cur_pos[2] += offset_diff
            # move the toolhead by the offset amount
            toolhead.move(cur_pos, 5.)
            logging.info(
                "temperature_probe: %.4f mm offset applied to probed position"
                % (offset_diff))
        else:
            # set the toolheads position by the offset amount
            cur_pos[2] += offset
            toolhead.set_position(cur_pos)
            self.homed_offset = offset
            logging.info(
                "temperature_probe: %.4f mm offset applied to Z homed position"
                % (offset))
        self.is_probing = False
        start_pos = toolhead.get_position()
        self.deactivate_gcode.run_gcode_from_command()
        if toolhead.get_position()[:3] != start_pos[:3]:
            raise homing.CommandError(
                "Toolhead moved during probe deactivate_gcode script")
        self.mcu_endstop.home_finalize()
    def _check_heater_state(self):
        toolhead = self.printer.lookup_object('toolhead')
        extruder = toolhead.get_extruder().get_heater()
        bed = self.printer.lookup_object('heater_bed', None)
        reactor = self.printer.get_reactor()
        eventtime = reactor.monotonic()
        extr_on = extruder.get_status(eventtime)['target'] > 0.
        if bed is not None:
            bed_on = bed.get_status(eventtime)['target'] > 0.
        else:
            bed_on = False
        return extr_on or bed_on
    def _wait_for_temp(self, timeout, is_warming):
        reactor = self.printer.get_reactor()
        toolhead = self.printer.lookup_object('toolhead')
        eventtime = reactor.monotonic()
        endtime = eventtime + timeout if timeout else reactor.NEVER
        while self.printer_ready and eventtime <= endtime:
            toolhead.get_last_move_time()
            msg = "temperature_probe: %.2f/%.2f" % (self.get_temp(eventtime))
            if timeout:
                msg += " Time Remaining: %d seconds" % (endtime - eventtime)
            self.gcode.respond_info(msg)
            eventtime = reactor.pause(eventtime + 1.)
            if is_warming:
                if self.sensor_temp >= self.target:
                    return True
            elif self.sensor_temp <= self.target:
                return True
        return False
    cmd_PROBE_WAIT_help = "Pause until the probe reaches a temperature"
    def cmd_PROBE_WAIT(self, params):
        self.target = self.gcode.get_float('TARGET', params, 35., above=0.)
        timeout = self.gcode.get_int('TIMEOUT', params, 0, minval=0)
        if self._check_heater_state():
            # either bed or extruder is on, probe is warming
            self.gcode.respond_info("Waiting for probe to heat...")
            temp_acheived = self._wait_for_temp(timeout, True)
        else:
            # both bed and extruder are off, probe is cooling
            self.gcode.respond_info("Waiting for probe to cool...")
            temp_acheived = self._wait_for_temp(timeout, False)
        if temp_acheived:
            self.gcode.respond_info("Probe Temperature Achieved")
        else:
            self.gcode.respond_info("Wait for Probe Temp Timed Out")
        self.target = 0.

def load_config(config):
    tprobe = TemperatureProbe(config, TemperatureProbeEndstopWrapper(config))
    config.get_printer().add_object('probe', tprobe)
    return tprobe
