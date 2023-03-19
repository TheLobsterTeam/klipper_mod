# Z-Probe support
#
# Copyright (C) 2017-2021  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging
import pins
from probe import ProbeEndstopWrapper, PrinterProbe
from extras.lobster_log import log, trace, signpost


class LobsterProbe(PrinterProbe):
    def __init__(self, config, mcu_probe):
        PrinterProbe.__init__(self, config, mcu_probe)
        self.retract_dist = config.getfloat("retract_safe_dist")
        self.probe_safe_height = config.getfloat("probe_safe_height")
        self.rails_dict = {}
        self.active_rail = None

    def register_rail(self, railname, steppers):
        stepper_list = [s.get_name() for s in self.mcu_probe.get_all_steppers()]
        for stepper in steppers:
            if stepper.get_name() not in stepper_list:
                self.mcu_probe.add_stepper(stepper)
        self.rails_dict[railname] = [s.get_name() for s in steppers]

    def set_active_rail(self, railname):
        if railname is None:
            self.active_rail = None
            self.mcu_probe.set_active_steppers(None)
            log("Rail set to none: %s"%railname, 'r')
            return

        if railname not in self.rails_dict.keys():
            raise Exception("Rail %s is not a registered rail"%railname)

        self.active_rail = railname
        self.mcu_probe.set_active_steppers(self.rails_dict[self.active_rail])
        log("Rail set to %s, %s"%(railname, self.active_rail), 'r')

    def _probe(self, speed):
        if self.active_rail is None:
            log("No active rail!! %s"%self.active_rail, 'r')
            raise Exception("No active rail!")

        return PrinterProbe._probe(self, speed)

    def probe_point(self, sample_count=None):
        sample_count = self.sample_count if sample_count is None else sample_count
        speed = self.speed
        lift_speed = self.get_lift_speed()
        sample_retract_dist = self.sample_retract_dist
        samples_tolerance = self.samples_tolerance
        samples_retries = self.samples_retries
        samples_result = self.samples_result
        must_notify_multi_probe = not self.multi_probe_pending
        if must_notify_multi_probe:
            self.multi_probe_begin()
        probexy = self.printer.lookup_object('toolhead').get_position()[:2]
        retries = 0
        positions = []
        while len(positions) < sample_count:
            # Probe position
            pos = self._probe(speed)
            logging.info(pos)
            positions.append(pos)
            # Check samples tolerance
            z_positions = [p[2] for p in positions]
            if max(z_positions) - min(z_positions) > samples_tolerance:
                if retries >= samples_retries:
                    raise logging.error("Probe samples exceed samples_tolerance")
                logging.info("Probe samples exceed tolerance. Retrying...")
                retries += 1
                positions = []
            # Retract
            if len(positions) < sample_count:
                self._move(probexy + [pos[2] + sample_retract_dist], lift_speed)
        if must_notify_multi_probe:
            self.multi_probe_end()

        # Calculate and return result
        if samples_result == 'median':
            result = self._calc_median(positions)
        else:
            result = self._calc_mean(positions)

        logging.info(result[2])
        self._move(probexy + [result[2] + self.retract_dist], lift_speed)  # finish
        return result[2]

    def safe_height(self):
        return self.probe_safe_height

# Endstop wrapper that enables probe specific features
class LobsterProbeEndstopWrapper(ProbeEndstopWrapper):
    def __init__(self, config):
        ProbeEndstopWrapper.__init__(self, config)
        self.stepper_dict = {}
        self.active_steppers = []

        # Wrappers
        self.get_all_steppers = self.mcu_endstop.get_steppers

        # Overwrite wrappers from super class
        self.add_stepper = self._add_stepper
        self.get_steppers = self._get_steppers

    def _handle_mcu_identify(self):
        pass
        # logging.info("sike lmao")
    def _add_stepper(self, stepper):
        self.mcu_endstop.add_stepper(stepper)
        self.stepper_dict[stepper.get_name()] = stepper

    def _get_steppers(self):
        return [self.stepper_dict[stepper] for stepper in self.active_steppers]

    def set_active_steppers(self, steppers):
        if steppers is None:
            self.active_steppers = None
            return

        for stepper in steppers:
            if stepper not in self.stepper_dict.keys():
                raise Exception("Stepper %s not registered"%stepper)
        self.active_steppers = steppers


def load_config(config):
    return LobsterProbe(config, LobsterProbeEndstopWrapper(config))
