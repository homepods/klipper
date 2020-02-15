# Helper for calibrating X and Y endstops
#
# Copyright (C) 2020  Eric Callahan <arksine.code@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.

class XYCalibrate:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.gcode = self.printer.lookup_object('gcode')
        self.steppers = {}
        x_home_dir = config.getboolean('x_homing_dir_positive', True)
        y_home_dir = config.getboolean('y_homing_dir_positive', True)
        self.homing_dirs = {'x': x_home_dir, 'y': y_home_dir}
        self.axes = config.get('axes', 'xy').lower()
        if self.axes not in ["x", "y", "xy"]:
            raise config.error(
                "Invalid axes option, must be one of x, y or xy")
        p_cfg = config.getsection('printer')
        kinematics = p_cfg.get('kinematics')
        if kinematics not in ['cartesian', 'corexy']:
            raise config.error(
                'XYCalibrate requires cartesian or corexy kinematics')
        self.has_homing_override = config.has_section('homing_override')
        self.printer.register_event_handler('klippy:connect',
                                            self._handle_connect)
        self.printer.register_event_handler('klippy:ready',
                                            self._handle_ready)
        self.gcode.register_command(
            'XY_ENDSTOP_CALIBRATE', self.cmd_XY_ENDSTOP_CALIBRATE,
            desc=self.cmd_XY_ENDSTOP_CALIBRATE_help)

    def _handle_connect(self):
        force_move = self.printer.lookup_object('force_move')
        for a in ['x', 'y']:
            stepper = force_move.lookup_stepper("stepper_" + a)
            self.steppers[a] = stepper

    def _handle_ready(self):
        # Workaround to initialize the MCU position offset for the
        # X and Y steppers
        for s in self.steppers.values():
            s.note_homing_end(True)

    def _calibrate_endstops(self, axes):
        configfile = self.printer.lookup_object('configfile')
        deltas = []
        for a in axes:
            start = self.steppers[a].get_mcu_position()
            self.gcode.run_script_from_command("G28 %s\nM400" % (a.upper()))
            end = self.steppers[a].get_mcu_position()
            delta = abs(end - start) * self.steppers[a].get_step_dist()
            deltas.append((a, delta))

        msg = "XY Endstop Calibration Results:"
        for (a, d) in deltas:
            if self.homing_dirs[a]:
                min_max_option = "position_max"
            else:
                d = -d
                min_max_option = "position_min"
            s_name = "stepper_%s" % (a)
            msg += "\n[%s]" % (s_name)
            msg += "\nposition_endstop: %.3f" % (d)
            msg += "\n%s: %.3f" % (min_max_option, d)
            configfile.set(s_name, 'position_endstop', "%.3f" % (d))
            configfile.set(s_name, min_max_option, "%.3f" % (d))
        msg += "\nThe SAVE_CONFIG command will update the printer config" \
               "file\n with the above and restart the printer."
        self.gcode.respond_info(msg)

    cmd_XY_ENDSTOP_CALIBRATE_help = "Calibrate X/Y Endstop Positions"
    def cmd_XY_ENDSTOP_CALIBRATE(self, params):
        if self.gcode.get_int("CHECK", params, 1) and self.has_homing_override:
            self.gcode.respond_info(
                "WARNING: [homing_override] has been detected printer.cfg.\n"
                "If you are sure that it will not interfere with homing\n"
                "the individual X and/or Y axes, you may proceed by adding\n"
                "'CHECK=0' to the XY_ENDSTOP_CALIBRATE gcode.")
            return
        axes = self.gcode.get_str("AXES", params, self.axes).lower()
        if axes not in ["x", "y", "xy"]:
            self.gcode.respond_info(
                "Invalid AXES parameter, must be one of X, Y or XY")
            return
        # Calibrate endstops works for corexy printers because we can assume
        # that each stepper is traveling the same distance on homing moves.
        # Thus calculating the distance for either the A or B stepper will
        # provide the desired result.
        self._calibrate_endstops(axes)

def load_config(config):
    return XYCalibrate(config)
