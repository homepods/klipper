# PAT9125 Filament Sensor
#
# Copyright (C) 2018  Eric Callahan <arksine.code@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.

import bus
import logging
import threading
import collections
from filament_switch_sensor import BaseSensor
import kinematics.extruder

CHIP_ADDR = 0x75
PAT9125_PRODUCT_ID = (0x91 << 8) | 0x31
FLAG_MOTION = 0x80

PAT9125_REGS = {
    'PID1': 0x00, 'PID2': 0x01, 'MOTION': 0x02,
    'DELTA_XL': 0x03, 'DELTA_YL': 0x04, 'MODE': 0x05,
    'CONFIG': 0x06, 'WP': 0x09, 'SLEEP1': 0x0a,
    'SLEEP2': 0x0b, 'RES_X': 0x0d, 'RES_Y': 0x0e,
    'DELTA_XYH': 0x12, 'SHUTTER': 0x14, 'FRAME': 0x17,
    'ORIENTATION': 0x19, 'BANK_SELECTION': 0x7f
}

PAT9125_INIT1 = [
    PAT9125_REGS['WP'], 0x5a,
    PAT9125_REGS['RES_X'], 0,
    PAT9125_REGS['RES_Y'], 240,
    PAT9125_REGS['ORIENTATION'], 0x04,
    0x5e, 0x08, 0x20, 0x64,
    0x2b, 0x6d, 0x32, 0x2f
]

PAT9125_INIT2 = [
    [0x06, 0x28, 0x33, 0xd0, 0x36, 0xc2, 0x3e, 0x01,
     0x3f, 0x15, 0x41, 0x32, 0x42, 0x3b, 0x43, 0xf2],
    [0x44, 0x3b, 0x45, 0xf2, 0x46, 0x22, 0x47, 0x3b,
     0x48, 0xf2, 0x49, 0x3b, 0x4a, 0xf0, 0x58, 0x98],
    [0x59, 0x0c, 0x5a, 0x08, 0x5b, 0x0c, 0x5c, 0x08,
     0x61, 0x10, 0x67, 0x9b, 0x6e, 0x22, 0x71, 0x07],
    [0x72, 0x08]
]

def check_twos_complement(val, bitsize):
    if val & (1 << (bitsize - 1)):
        val -= (1 << bitsize)
    return val

# I2C communications layer for pat9125
class PAT9125_I2C:
    RETRY_TIME = .25
    TIMEOUT = .75
    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.i2c = bus.MCU_I2C_from_config(
            config, default_addr=CHIP_ADDR, default_speed=400000)
        self.mcu = self.i2c.get_mcu()

        self.oid = self.mcu.create_oid()
        self.get_i2c_oid = self.i2c.get_oid
        self.get_command_queue = self.i2c.get_command_queue
        self.get_mcu = self.i2c.get_mcu
        self.pat9125_write_verify_cmd = None

        self.last_query_time = 0.
        self.write_verify_response = None
        self.write_verify_request = None
        self.write_verify_timer = self.reactor.register_timer(
            self._write_verify_request_event)
    def build_config(self):
        self.pat9125_write_verify_cmd = self.mcu.lookup_command(
            "command_pat9125_write_verify oid=%c sequence=%*s retries=%u",
            cq=self.get_command_queue())
        self.mcu.register_response(
            self._response_handler, "pat9125_verify_response", self.oid)
    def _write_verify_request_event(self, eventtime):
        if self.write_verify_response is not None:
            return self.reactor.NEVER
        self.pat9125_write_verify_cmd.send(
            self.write_verify_request)
        return eventtime + self.RETRY_TIME
    def _response_handler(self, params):
        if params['#sent_time'] >= self.last_query_time:
            self.write_verify_response = params['success']
    def get_oid(self):
        return self.oid
    def read_register(self, reg, read_len):
        # return data from from a register. reg may be a named
        # register, an 8-bit address, or a list containing the address
        if type(reg) is str:
            regs = [PAT9125_REGS[reg]]
        elif type(reg) is list:
            regs = list(reg)
        else:
            regs = [reg]
        params = self.i2c.i2c_read(regs, read_len)
        return bytearray(params['response'])
    def write_register(self, reg, data, minclock=0, reqclock=0):
        # Write data to a register. Reg may be a named
        # register or an 8-bit address.  Data may be a list
        # of 8-bit values, or a single 8-bit value
        if type(data) is not list:
            out_data = [data & 0xFF]
        else:
            out_data = list(data)

        if type(reg) is str:
            out_data.insert(0, PAT9125_REGS[reg])
        elif type(reg) is list:
            out_data = reg + out_data
        else:
            out_data.insert(0, reg)
        self.i2c.i2c_write(out_data, minclock, reqclock)
    def write_verify_sequence(self, sequence, retries=5):
        # do a register read/write.  We need to handle our own response
        # because 'send_with_response' uses a different command queue
        # that alters the order of commands sent
        self.write_verify_response = None
        self.write_verify_request = [self.oid, sequence, retries]
        self.reactor.update_timer(self.write_verify_timer, self.reactor.NOW)
        eventtime = self.last_query_time = self.reactor.monotonic()
        while self.write_verify_response is None:
            eventtime = self.reactor.pause(eventtime + .005)
            if eventtime > self.last_query_time + self.TIMEOUT:
                break
        self.reactor.update_timer(self.write_verify_timer, self.reactor.NEVER)
        if self.write_verify_response is not None:
            return self.write_verify_response
        else:
            logging.info("pat9125: Timeout waiting for response")
            return False

DetectMode = {"OFF": 0, "RUNOUT": 1, "INSERT": 2}

class PAT9125(BaseSensor):
    def __init__(self, config):
        super(PAT9125, self).__init__(config)
        self.reactor = self.printer.get_reactor()
        self.pat9125_i2c = PAT9125_I2C(config)
        self.mcu = self.pat9125_i2c.get_mcu()
        self.mcu.register_config_callback(self._build_config)
        self.cmd_queue = self.pat9125_i2c.get_command_queue()
        self.oid = self.pat9125_i2c.get_oid()
        self.i2c_oid = self.pat9125_i2c.get_i2c_oid()
        self.printer.register_event_handler("klippy:ready", self._handle_ready)
        self.printer.register_event_handler(
            "gcode:request_restart", self._handle_restart)

        self.stepper_name = config.get("stepper", "extruder").lower()
        if not config.has_section(self.stepper_name):
            raise config.error(
                "pat9125: Stepper name [%s] not in configuration" %
                (self.stepper_name))
        self.stepper_oid = None
        self.pat9125_start_update_cmd = None
        self.pat9125_stop_update_cmd = None

        # TODO: Calculate a sane default for runout_refresh_time using the
        # nozzle size and cross section.We want to be able to poll fast enough
        # to cover .2mm - .5mm of filament movement at the maximum possible
        # extrusion speed. For example, extruding PLA with a .4 nozzle at
        # 185mm/sec needs to refresh approximately .08 seconds to cover .5mm
        # of movement.  A larger nozzle with a bigger cross_section may be able
        # to double that, which  may not be the best candidate for using a
        # PAT9125 to detect filament loss
        self.ro_ref_time = config.getfloat(
            "runout_refresh_time", .07, above=0., maxval=1.)
        self.ins_ref_time = config.getfloat(
            "insert_refresh_time", .04, above=0., maxval=1.)
        self.invert_axis = config.getboolean("invert_axis", False)
        self.oq_enable = int(config.getboolean("oq_enable", False))
        orient_choices = {'X': 0, 'Y': 1}
        self.orientation = config.getchoice(
            "orientation", orient_choices, 'Y')
        self.xres = 0 if self.orientation else 240
        self.yres = 240 if self.orientation else 0
        PAT9125_INIT1[3] = self.xres
        PAT9125_INIT1[5] = self.yres

        self.tracker = MotionTracker(config, self)
        self.sensor_mode = DetectMode["OFF"]
        self.detect_enabled = self.sensor_enabled = True
        self.initialized = False
        self.e_step_dist = 1.
        self.pat9125_state = {
            'STEPPER_POS': 0,
            'X_POS': 0,
            'Y_POS': 0,
            'FRAME': 0,
            'SHUTTER': 0,
            'MOTION': False
        }

        self.gcode.register_mux_command(
            "QUERY_FILAMENT_SENSOR", "SENSOR", self.name,
            self.cmd_QUERY_FILAMENT_SENSOR,
            desc=self.cmd_QUERY_FILAMENT_SENSOR_help)
        self.gcode.register_mux_command(
            "SET_FILAMENT_SENSOR", "SENSOR", self.name,
            self.cmd_SET_FILAMENT_SENSOR,
            desc=self.cmd_SET_FILAMENT_SENSOR_help)
    def _handle_ready(self):
        self._init_stepper()
        self._pat9125_init()
        if self.insert_enabled:
            self.set_mode("INSERT")
    def _handle_restart(self, print_time):
        # stop the pat9125 mcu timer
        self.detect_enabled = False
        self.insert_enabled = False
        self.runout_enabled = False
        try:
            self.set_mode("OFF")
        except Exception as e:
            logging.info("pat9125: Error attempting to stop on restart")
            logging.info(str(e))
    def _build_config(self):
        self.mcu.add_config_cmd(
            "command_config_pat9125 oid=%d i2c_oid=%d oq_enable=%d"
            % (self.oid, self.i2c_oid, self.oq_enable))
        self.pat9125_start_update_cmd = self.mcu.lookup_command(
            "command_pat9125_start_update oid=%c step_oid=%c clock=%u "
            "rest_ticks=%u", cq=self.cmd_queue)
        self.pat9125_stop_update_cmd = self.mcu.lookup_command(
            "command_pat9125_stop_update oid=%c", cq=self.cmd_queue)
        self.pat9125_i2c.build_config()
    def _init_stepper(self):
        # XXX - need to add get_stepper() function to extruder class
        # and manual stepper class
        stepper = None
        if self.stepper_name == "extruder":
            self.stepper_name = "extruder0"
        obj = self.printer.lookup_object(self.stepper_name, None)
        if obj is not None and hasattr(obj, "stepper"):
            stepper = obj.stepper
        else:
            # parent is not a printer object (ie extruder or manual stepper)
            # check the toolhead steppers
            toolhead = self.printer.lookup_object('toolhead')
            stepper_list = toolhead.get_kinematics().get_steppers()
            for s in stepper_list:
                if self.stepper_name == s.get_name():
                    stepper = s
        if stepper is None:
            # XXX - should probably raise a different error here
            raise self.gcode.error(
                "pat9125: Unable to locate stepper [%s]" % (self.stepper_name))
        else:
            self.set_stepper(stepper)
    def _pat9125_init(self):
        mcu = self.mcu
        self.initialized = False

        # make sure the mcu timer is stopped
        self.set_mode("OFF")

        # Read and verify product ID
        if not self._check_product_id():
            logging.info("Rechecking ID...")
            if not self._check_product_id():
                return

        self.pat9125_i2c.write_register('BANK_SELECTION', 0x00)
        print_time = mcu.estimated_print_time(self.reactor.monotonic())
        minclock = mcu.print_time_to_clock(print_time + .001)
        self.pat9125_i2c.write_register('CONFIG', 0x97, minclock=minclock)
        minclock = mcu.print_time_to_clock(print_time + .002)
        self.pat9125_i2c.write_register('CONFIG', 0x17, minclock=minclock)

        if not (self.pat9125_i2c.write_verify_sequence(PAT9125_INIT1)):
            logging.info("pat9125: Init Sequence 1 failure")
            return

        print_time = mcu.estimated_print_time(self.reactor.monotonic())
        minclock = mcu.print_time_to_clock(print_time + .01)
        self.pat9125_i2c.write_register(
            'BANK_SELECTION', 0x01, minclock=minclock)
        for sequence in PAT9125_INIT2:
            if not self.pat9125_i2c.write_verify_sequence(sequence):
                logging.info("pat9125: Init Sequence 2 failure")
                self.pat9125_i2c.write_register('BANK_SELECTION', 0x00)
                return

        self.pat9125_i2c.write_register('BANK_SELECTION', 0x00)
        if not self.pat9125_i2c.write_verify_sequence(
                [PAT9125_REGS['WP'], 0x00]):
            logging.info("pat9125: Unable to re-enable write protect")
            return

        if not self._check_product_id():
            return

        self.pat9125_i2c.write_register('RES_X', self.xres)
        self.pat9125_i2c.write_register('RES_Y', self.yres)
        self.initialized = True

        self.pat9125_state['X_POS'] = 0
        self.pat9125_state['Y_POS'] = 0
        self.tracker.reset()
        self.mcu.register_response(
            self._handle_pat9125_update, "pat9125_update_response", self.oid)

        logging.debug("pat9125: Initialization Success")
    def _handle_pat9125_update(self, params):
        if params['flags'] & 0x01 == 0:
            # No ack / comms error
            self.reactor.register_async_callback(
                (lambda e, s=self: s._handle_comms_error()))
            return

        self._pat9125_parse_status(
            bytearray(params['data']), params['flags'] & FLAG_MOTION)
        self.pat9125_state['STEPPER_POS'] = params['epos']

        self.reactor.register_async_callback(
            (lambda e, s=self, p=dict(self.pat9125_state):
                s.tracker.update(e, p)))
    def _handle_comms_error(self):
        self.initialized = False
        self.detect_enabled = False
        self.insert_enabled = False
        self.runout_enabled = False
        logging.info(
            "pat9125: Communication Error encountered, monitoring stopped")
    def _pat9125_parse_status(self, data, motion):
        self.pat9125_state['MOTION'] = motion
        if motion:
            xl = data[0]
            yl = data[1]
            xyh = data[2]
            dx = check_twos_complement((xl | ((xyh << 4) & 0xF00)), 12)
            dy = check_twos_complement((yl | ((xyh << 8) & 0xF00)), 12)
            if self.invert_axis:
                self.pat9125_state['Y_POS'] -= dy
                self.pat9125_state['X_POS'] -= dx
            else:
                self.pat9125_state['Y_POS'] += dy
                self.pat9125_state['X_POS'] += dx
        if self.oq_enable:
            self.pat9125_state['SHUTTER'] = data[3]
            self.pat9125_state['FRAME'] = data[4]
    def _check_product_id(self):
        pid = self.pat9125_i2c.read_register('PID1', 2)
        if ((pid[1] << 8) | pid[0]) != PAT9125_PRODUCT_ID:
            logging.info(
                "pat9125: Product ID Mismatch Expected: %d, Recd: %d"
                % (PAT9125_PRODUCT_ID, ((pid[1] << 8) | pid[0])))
            return False
        return True
    def set_stepper(self, stepper):
        # XXX - need to add get_oid to stepper class
        self.e_step_dist = stepper.get_step_dist()
        self.stepper_oid = stepper.mcu_stepper.get_oid()
        self.tracker.set_runout_limits(self.e_step_dist)
    def set_mode(self, new_mode):
        if (self.sensor_mode == DetectMode[new_mode] or
                not self.initialized):
            return

        self.sensor_mode = DetectMode[new_mode]
        if self.sensor_mode == DetectMode["RUNOUT"]:
            refresh_time = self.ro_ref_time
        elif self.sensor_mode == DetectMode["INSERT"]:
            refresh_time = self.ins_ref_time
        elif self.sensor_mode == DetectMode["OFF"]:
            self.pat9125_stop_update_cmd.send([self.oid])
            return
        else:
            self.sensor_mode = DetectMode["OFF"]
            # XXX - Probably shouldn't be a gcode error
            raise self.gcode.error(
                "pat9125: Unknown mode [%s]" % (new_mode))

        # Start PAT9125 read timer
        clock = self.mcu.get_query_slot(self.oid)
        rest_ticks = self.mcu.seconds_to_clock(refresh_time)
        self.pat9125_start_update_cmd.send(
            [self.oid, self.stepper_oid, clock, rest_ticks])
        self.tracker.reset()
        logging.debug(
            "pat9125: Polling activated, sampling every %.4f ms"
            % (refresh_time))
    def get_state(self):
        return dict(self.pat9125_state)
    def set_enable(self, runout, insert):
        super(PAT9125, self).set_enable(runout, insert)
        if self.detect_enabled:
            if runout or insert:
                mode = "INSERT" if insert else "RUNOUT"
            else:
                mode = "OFF"
            self.set_mode(mode)
        else:
            self.runout_enabled = self.insert_enabled = False
    def cmd_QUERY_FILAMENT_SENSOR(self, params):
        state = self.get_state()
        if state['MOTION']:
            msg = "pat9125 sensor: Motion Detected"
        else:
            msg = "pat9125 sensor: Motion not Detected"
        msg += "\nCurrent Position: X=%d Y=%d STEPPER=%.6f" % (
            state['X_POS'], state['Y_POS'],
            state['STEPPER_POS'] * self.e_step_dist)
        if self.oq_enable:
            msg += "\nAverage Brightness: %d Laser Shutter Time: %d" % (
                state['FRAME'], state['SHUTTER'])
        self.gcode.respond_info(msg)
    def cmd_SET_FILAMENT_SENSOR(self, params):
        self.sensor_enabled = self.gcode.get_int("ENABLE", params, 1)

XYE_KEYS = ['X_POS', 'Y_POS', 'STEPPER_POS']

class MotionTracker:
    def __init__(self, config, pat9125):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.pat9125 = pat9125
        self.oq_enable = pat9125.oq_enable
        self.orientation = pat9125.orientation
        self.event_delay = config.getfloat('event_delay', 3., above=0.)
        self.need_xye_init = True
        self.last_event_time = 0.
        self.last_xye = [0, 0, 0]
        self.oq_deque = collections.deque(maxlen=20)

        # runout tracking
        self.max_runout_errors = config.getint(
            'max_runout_errors', 17, minval=1)
        self.max_blind_movement = config.getfloat(
            'max_blind_movement', 2., above=0.)
        self.blind_error_distance = config.getfloat(
            'blind_error_distance', .3, above=0.)
        self.ro_total_errors = 0
        self.ro_blind_errors = 0
        self.ro_prev_d = 0
        self.ro_blind_steps = 0

        self.max_blind_steps = 0
        self.min_error_steps = 0

        # insert tracking
        self.max_insert_counter = config.getint(
            'max_insert_counter', 12, minval=1)
        self.max_insert_sum = config.getint(
            'max_insert_sum', 20, minval=1)
        self.ins_sum = 0
        self.ins_change_counter = 0
    def _log_oq_results(self):
        if self.oq_enable:
            logging.debug(
                "pat9125: Last 20 OQ results (shutter, frame) %s" %
                str(list(self.oq_deque)))
    def set_runout_limits(self, step_distance):
        self.max_blind_steps = int(self.max_blind_movement / step_distance + .5)
        self.min_error_steps = int(
            self.blind_error_distance / step_distance + .5)
    def reset(self):
        self.need_xye_init = True
        self.ro_total_errors = self.ro_blind_errors = 0
        self.ro_blind_steps = self.ro_prev_d = 0
        self.ins_sum = self.ins_change_counter = 0
    def update(self, eventtime, state):
        curpos = [state[key] for key in XYE_KEYS]
        delta = [curpos[i] - self.last_xye[i] for i in range(3)]
        self.oq_deque.append((state['SHUTTER'], state['FRAME']))
        self.last_xye[:] = curpos
        if self.need_xye_init:
            self.need_xye_init = False
            return

        if not self.pat9125.sensor_enabled:
            return

        if self.pat9125.runout_enabled:
            if eventtime - self.last_event_time < self.event_delay:
                return
            if self.check_runout(delta[self.orientation], delta[2]):
                self.last_event_time = eventtime
                self._log_oq_results()
        elif self.pat9125.insert_enabled:
            if eventtime - self.last_event_time < self.event_delay:
                return
            if self.check_insert(delta[self.orientation]):
                self.last_event_time = eventtime
                self._log_oq_results()
    def check_runout(self, axis_d, stepper_d):
        if stepper_d > 0:
            # extruder is moving in the positive direction
            if axis_d < -5:
                # Significant negative delta.  The stepper is
                # moving forward, axis is moving backward.  This
                # could be a jam.
                logging.debug(
                    "pat9125: Negative motion detected,"
                    " Axis Delta: %d Stepper Delta %d" %
                    (axis_d, stepper_d))
                self.ro_blind_steps += stepper_d
                if self.ro_total_errors:
                    self.ro_total_errors += 2
                else:
                    self.ro_total_errors += 1
            elif axis_d > 0:
                # positive movement, we are good
                self.ro_blind_steps = self.ro_blind_errors = 0
                if self.ro_total_errors:
                    self.ro_total_errors -= 1
            else:
                # No movement detected, we are stepping blind.
                self.ro_blind_steps += stepper_d
                cur_err = self.ro_blind_steps / (self.ro_blind_errors + 1)
                if ((self.ro_prev_d <= 0 or self.ro_total_errors) and
                        cur_err > self.min_error_steps):
                    self.ro_total_errors += 1
                    self.ro_blind_errors += 1

        self.ro_prev_d = axis_d
        # XXX - Possible to include frame shutter time as a backup
        # method for verifying a runout.  Generally the shutter
        # time adjusts to a lower number if the object being tracked
        # contrasts with the background.  The empty shutter value
        # would need to be calibrated
        if (self.ro_total_errors > self.max_runout_errors or
                self.ro_blind_steps > self.max_blind_steps):
            logging.info(
                "pat9125: Runout Detected - Error Count: %d Blind Steps: %d" %
                (self.ro_total_errors, self.ro_blind_steps))
            self.ro_total_errors = self.ro_blind_errors = 0
            self.ro_blind_steps = self.ro_prev_d = 0
            self.reactor.register_callback(self.pat9125._runout_event_handler)
            return True
        return False
    def check_insert(self, axis_d):
        if axis_d != 0:
            if axis_d > 0:
                self.ins_sum += axis_d
                self.ins_change_counter += 3
            elif self.ins_change_counter > 1:
                self.ins_change_counter -= 2
        elif self.ins_change_counter > 0:
            self.ins_change_counter -= 1

        if not self.ins_change_counter:
            self.ins_sum = 0

        if (self.ins_change_counter >= self.max_insert_counter and
                self.ins_sum > self.max_insert_sum):
            logging.info("pat9125: Insert detected")
            self.ins_sum = self.ins_change_counter = 0
            self.reactor.register_callback(self.pat9125._insert_event_handler)
            return True
        return False

def load_config_prefix(config):
    return PAT9125(config)
