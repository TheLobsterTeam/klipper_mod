# Support for servos
#
# Copyright (C) 2017-2020  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.

SPINDLE_SIGNAL_PERIOD = 0.020
PIN_MIN_TIME = 0.100


class Spindle:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.min_width = config.getfloat('minimum_pulse_width', .001,
                                         above=0., below=SPINDLE_SIGNAL_PERIOD)
        self.max_width = config.getfloat('maximum_pulse_width', .002,
                                         above=self.min_width,
                                         below=SPINDLE_SIGNAL_PERIOD)
        self.max_rpm = config.getfloat('maximum_rpm')
        self.angle_to_width = (self.max_width - self.min_width) / self.max_rpm
        self.width_to_value = 1. / SPINDLE_SIGNAL_PERIOD
        self.last_value = self.last_value_time = 0.
        initial_pwm = 0.
        iangle = config.getfloat('initial_rpm', None, minval=0., maxval=360.)
        if iangle is not None:
            initial_pwm = self._get_pwm_from_rpm(iangle)
        else:
            iwidth = config.getfloat('initial_pulse_width', 0.,
                                     minval=0., maxval=self.max_width)
            initial_pwm = self._get_pwm_from_pulse_width(iwidth)

        # Setup mcu_spindle pin
        ppins = self.printer.lookup_object('pins')
        self.mcu_spindle = ppins.setup_pin('pwm', config.get('pin'))
        self.mcu_spindle.setup_max_duration(0.)
        self.mcu_spindle.setup_cycle_time(SPINDLE_SIGNAL_PERIOD)
        self.mcu_spindle.setup_start_value(initial_pwm, 0.)

        # Register commands
        spindle_name = config.get_name().split()[1]
        gcode = self.printer.lookup_object('gcode')
        gcode.register_mux_command("SET_SPINDLE", "SPINDLE", spindle_name,
                                   self.cmd_SET_SPINDLE,
                                   desc=self.cmd_SET_SPINDLE_help)

    def get_status(self, eventtime):
        return {'value': self.last_value}

    def set_rpm(self, print_time, rpm):
        self._set_pwm(print_time, self._get_pwm_from_rpm(rpm))

    def _set_pwm(self, print_time, value):
        if value == self.last_value:
            return
        print_time = max(print_time, self.last_value_time + PIN_MIN_TIME)
        self.mcu_spindle.set_pwm(print_time, value)
        self.last_value = value
        self.last_value_time = print_time

    def _get_pwm_from_rpm(self, rpm):
        rpm = max(0., min(self.max_rpm, rpm))
        width = self.min_width + rpm * self.angle_to_width
        return width * self.width_to_value

    def _get_pwm_from_pulse_width(self, width):
        if width:
            width = max(self.min_width, min(self.max_width, width))
        return width * self.width_to_value

    cmd_SET_SPINDLE_help = "Set spindle rpm"
    def cmd_SET_SPINDLE(self, gcmd):
        print_time = self.printer.lookup_object('toolhead').get_last_move_time()
        width = gcmd.get_float('WIDTH', None)
        if width is not None:
            self._set_pwm(print_time, self._get_pwm_from_pulse_width(width))
        else:
            rpm = gcmd.get_float('RPM')
            self.set_rpm(print_time, rpm)


def load_config_prefix(config):
    return Spindle(config)
