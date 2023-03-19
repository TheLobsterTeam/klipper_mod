# Code for handling the kinematics of cartesian robots
#
# Copyright (C) 2016-2021  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging

import numpy
from typing import List

import gcode
import klippy
import stepper

from extras import manual_stepper, spindle, lobster_probe, lobster_net, gcode_move
from extras.homing import Homing
from extras.lobster_log import log, signpost, trace
from extras.lobster_probe import LobsterProbe, LobsterProbeEndstopWrapper


class LobKinematics:
    def __init__(self, toolhead, config):
        # Printer object
        self.printer = config.get_printer()  # type: klippy.Printer
        self.toolhead = toolhead  # type: toolhead.ToolHead
        self.feed_rate = self.toolhead.get_max_velocity()

        # Gcode Command Registration
        gc = self.printer.lookup_object('gcode')  # type: gcode.GCodeDispatch
        gc.register_command('M06', self.cmd_M06)  # Tool Change
        gc.register_command('M05', self.cmd_M05)  # Stop Spindle
        gc.register_command('M04', self.cmd_M04)  # CCW Spindle
        gc.register_command('M03', self.cmd_M03)  # CW Spindle
        gc.register_command('M42069', self.cmd_M42069)  # Flip PCNB
        gc.register_command('INIT_LOB', self.cmd_INIT_LOB)  # Initialize Printer
        gc.register_command('S', self.cmd_S)  # Set Spindle Speed
        gc.register_command('F', self.cmd_F)  # Set Feedrate

        # Define Rails and corresponding axes
        self.railnames = ["pcb_x", "wire_x", "pcb_y", "spindle1_z", "spindle2_z", "spindle3_z", "wire_z"]
        self.railaxes = ['x', 'x', 'y', 'z', 'z', 'z', 'z']

        # Rails Setup
        self.rails = {}  # Rails Dictionary
        self.rail_dic = {}
        self.material_origin = [config.getfloat("material_origin_x"), config.getfloat("material_origin_y")]
        self.tool_origins = []
        for name, axis in zip(self.railnames, self.railaxes):  # Adding PCB Axes
            rail_section = config.getsection(name)

            rail = stepper.LookupMultiRail(rail_section)
            rail.setup_itersolve('cartesian_stepper_alloc', axis.encode())

            for s in rail.get_steppers():  # Register steppers with toolhead
                toolhead.register_step_generator(s.generate_steps)
            if axis == 'z':
                offsets = [rail_section.getfloat("offset_x"), rail_section.getfloat("offset_y")]
                self.tool_origins.append([self.material_origin[0] + offsets[0],
                                          self.material_origin[1] + offsets[1]])

            rail_dic = {
                'rail': rail,
                'limits': (1.0, -1),
                'pos': rail.get_commanded_position()
            }
            self.rails[name] = rail_dic

        log("Tool Origins: %s" % self.tool_origins, 'b')

        self.active_tool = 0
        self.tool_configs = [["pcb_x", "pcb_y", "spindle1_z"],
                             ["pcb_x", "pcb_y", "spindle2_z"],
                             ["pcb_x", "pcb_y", "spindle3_z"],
                             ["wire_x", "pcb_y", "wire_z"]]
        self.tool_names = ["Spindle 1", "Spindle 2", "Spindle 3", "Wire Feed"]
        self.tool_limits = [(0, config.getfloat("material_width")), (0, config.getfloat("material_height"))]
        self._coords_local = False

        active_rails = self.get_active_rails()
        active_steppers = [s for rail in active_rails for s in rail.get_steppers()]
        for s in active_steppers:  # setup trapq for active rails
            s.set_trapq(toolhead.get_trapq())
        self.printer.register_event_handler("stepper_enable:motor_off",
                                            self._motor_off)

        # Setup boundary checks
        max_velocity, max_accel = toolhead.get_max_velocity()  # type: float, float
        self.max_z_velocity = config.getfloat('max_z_velocity', max_velocity,
                                              above=0., maxval=max_velocity)
        self.max_z_accel = config.getfloat('max_z_accel', max_accel,
                                           above=0., maxval=max_accel)
        self.limits = [(1.0, -1.0)] * 3
        ranges = [r.get_range() for r in active_rails]
        self.axes_min = toolhead.Coord(*[r[0] for r in ranges], e=0.)  # type: gcode.Coord
        self.axes_max = toolhead.Coord(*[r[1] for r in ranges], e=0.)  # type: gcode.Coord

        # Initialize Probe
        lprobe_config = config.getsection("lobster_probe")
        self.probe = self.printer.lookup_object("lobster_probe")  # type: lobster_probe.LobsterProbe
        self.probe_rails = lprobe_config.getlist("axes")
        for rail in self.probe_rails:
            self.probe.register_rail(rail, self.rails[rail]['rail'].get_steppers())
        self.probe.set_active_rail(self.tool_configs[self.active_tool][2])

        self.net = self.printer.lookup_object("lobster_net")  # type: lobster_net.LobsterNet

        # Get Spindles
        config_spindles = self.printer.lookup_objects("spindle")

        self.spindles = []  # type: List[spindle.Spindle]
        for s in ["spindle spindle%d" % n for n in [1, 2, 3]]:
            for conf_spindle in config_spindles:
                if s == conf_spindle[0]:
                    self.spindles.append(conf_spindle[1])
                    break

        # Initialize PCB_flip
        self.pcb_stepper = self.printer.lookup_object("manual_stepper pcb_flip")  # type: manual_stepper.ManualStepper

        self._coords_local = False
        self.move_id = 0
        self.pcb_side = -1
        self.interpolate = False
        self.interpolate_prev = False
        self.spindle_speed = 0
        self.spindle_on = False

        gc_move = self.printer.lookup_object("gcode_move") # type: gcode_move.GCodeMove
        gc_move.set_move_transform(self)

        # log("\"https://google.com\"   ", 'r'
        # self.printer.register_event_handler("klippy:ready", self._set_interp_transform())

    def move(self, newpos, speed):
        self._check_shutdown()
        path = []
        startpos = self.get_position()
        if self.interpolate:
            path = self.net.interp_path(self._active_pointmap(), startpos, newpos)
            path = [p + startpos[3:] for p in path]
            log("INTERPOLATED PATH: %s"%path, 'r')

        if len(path) == 0:
            path = [newpos]

        for pos in path:
            self.toolhead.move(pos, self.feed_rate)

    def manual_move(self, newpos, speed):
        self._check_shutdown()
        self.toolhead.manual_move(newpos, speed)

    def get_position(self):
        pos = self.toolhead.get_position()
        if self.interpolate:
            pos[2] -= self.net.get_interpolated(self._active_pointmap(), pos)
        # log("INTERCEPTED GET_POS CALL", 'r')
        return pos

    def get_active_rails(self):
        return [self.rails[name]['rail'] for name in self.tool_configs[self.active_tool]]

    # Return machine stepper motors in order of axis name
    def get_steppers(self):
        rails = [self.rails[name]['rail'] for name in self.railnames]
        return [s for rail in rails for s in rail.get_steppers()]

    # Calculate the position of the active rails
    def calc_position(self, stepper_positions):
        # signpost("CALC POSITION", 'pk')
        # log("%s"%(stepper_positions))
        # trace('pk')
        position = [stepper_positions[rail.get_name()] for rail in self.get_active_rails()]
        # log("Global: %s"%position)
        loc = tuple(position)
        position = self._transform_to_local(position)
        glob = tuple(position)
        log("Calc Position ==> Loc: %s, Glob: %s" % (loc, glob), 'gr')
        # log("Local: %s"%position)
        # trace(depth=-1)
        # signpost("CALC POSITION", 'pk')
        # if self.interpolate:
        #     position = self._interp(position, True)

        return position

    # Set the position of the active rails
    def set_position(self, newpos, homing_axes):
        # signpost("SET POSITION", 'pk')
        # trace('pk')
        # log("Local Pos: %s" % newpos, 'b')
        loc = list(newpos)
        newpos = self._transform_from_local(newpos)
        glob = list(newpos)
        log("Set Position ==> Loc: %s, Glob: %s" % (loc, glob), 'gr')
        # trace(depth=0)
        # if self.interpolate:
        #     newpos = self._interp(newpos)
        # log("Global Pos: %s" % newpos, 'b')

        for i, rail in enumerate(self.get_active_rails()):
            rail.set_position(newpos)
            if i in homing_axes:
                self.limits[i] = rail.get_range()

    # Set active Z rail to unhomed state
    def note_z_not_homed(self):
        # Helper for Safe Z Home
        self.limits[2] = (1.0, -1.0)

    # Home specific rail
    def _home_axis(self, homing_state, axis, rail):
        last_coords_state = self._set_coords_local(False)

        # Determine movement
        position_min, position_max = rail.get_range()
        hi = rail.get_homing_info()
        homepos = [None, None, None, None]
        homepos[axis] = hi.position_endstop
        forcepos = list(homepos)
        if hi.positive_dir:
            forcepos[axis] -= 1.5 * (hi.position_endstop - position_min)
        else:
            forcepos[axis] += 1.5 * (position_max - hi.position_endstop)
        # Perform homing
        homing_state.home_rails([rail], forcepos, homepos)

        self._set_coords_local(last_coords_state)

    # Home specified active rails in the order Z,Y,X
    def home(self, homing_state):
        last_coords_state = self._set_coords_local(False)

        # Each axis is homed independently and in order
        axes = homing_state.get_axes()
        for axis in [2, 1, 0]:  # Home axes in the order Z,Y,X
            if axis in axes:
                log("Homing axis %s on %s" % ("xyz"[axis], self.tool_names[self.active_tool]), 'gr')
                self._home_axis(homing_state, axis, (self.get_active_rails())[axis])

        self._set_coords_local(last_coords_state)

    # Turn off the active rails
    def _motor_off(self, print_time):
        self.limits = [(1.0, -1.0)] * 3

    # Check active rail endstops
    def _check_endstops(self, move):
        # log("Homing with limits enabled: %s"%self.local_tool_limits)
        end_pos = move.end_pos
        limits = self._get_limits()

        for i in (0, 1, 2):
            if (move.axes_d[i]
                    and (end_pos[i] < limits[i][0]
                         or end_pos[i] > limits[i][1])):
                if limits[i][0] > limits[i][1]:
                    raise move.move_error("Must home axis first")
                log("exceeded limits on axis (%s) ==> pos:%s delta:%s limits:%s" % (
                    "xyz"[i], end_pos[i], move.axes_d[i], limits[i]), 'r')
                raise move.move_error()

    # Check active axes
    def check_move(self, move):  # type: (LobKinematics, toolhead.Move) -> none
        self._check_shutdown()
        signpost("CHECKING MOVE", 'o')  # self.toolhead.get_last_move_time()
        log("Moving %s in %s space with interpolation=(%s, %s)" % (
            self.tool_names[self.active_tool],
            "Local" if self._coords_local else "Global", self.interpolate, self.interpolate_prev), 'b')
        log("Current Limits: x:%s y:%s z:%s" % (self.limits[0], self.limits[1], self.limits[2]), 'b')
        limits = self._get_limits()
        # trace()

        # Update move start and end to proper coordinate system
        log("Local Start: %s, Local End: %s:" % (move.start_pos, move.end_pos), 'y')

        # log("Uninterp Start: %s, Uninterp End: %s:"%(move.start_pos, move.end_pos), 'y')
        # if self._coords_local:
        #     if self.interpolate_prev:
        #         move.start_pos = tuple(self._interp(move.start_pos))
        #     if self.interpolate:
        #         move.end_pos = tuple(self._interp(move.end_pos))
        # self._set_interp()
        # log("Interp Start: %s, Interp End: %s:"%(move.start_pos, move.end_pos), 'y')

        xpos, ypos = move.end_pos[:2]
        if (xpos < limits[0][0] or xpos > limits[0][1]
                or ypos < limits[1][0] or ypos > limits[1][1]):
            self._check_endstops(move)

        if move.axes_d[2]:  # Move with Z - update velocity and accel for slower Z axis
            self._check_endstops(move)
            z_ratio = move.move_d / abs(move.axes_d[2])
            move.limit_speed(
                self.max_z_velocity * z_ratio, self.max_z_accel * z_ratio)
        else:
            pass  # Normal XY move - use defaults

        move.start_pos = tuple(self._transform_from_local(list(move.start_pos)))
        move.end_pos = tuple(self._transform_from_local(list(move.end_pos)))
        log("Global Start: %s, Global End: %s:" % (move.start_pos, move.end_pos), 'y')
        signpost("DONE MOVE CHECK", 'o')

    # Get status of active rails
    def get_status(self, eventtime):
        axes = [a for a, (l, h) in zip("xyz", self.limits) if l <= h]
        return {
            'homed_axes': "".join(axes),
            'axis_minimum': self.axes_min,
            'axis_maximum': self.axes_max,
        }

    # =================================================
    # Manual Hooks
    # =================================================
    def _check_shutdown(self):
        if self.printer.is_shutdown():
            self.printer.lookup_object('stepper_enable').motor_off()
            raise gcode.CommandError("Cannot complete command in shutdown state")

    def _active_pointmap(self):
        return self.tool_names[self.active_tool] + "S%d" % self.pcb_side

    def _build_mesh(self):
        self._check_shutdown()
        self._set_interp(False)
        self._manual_home([2])
        self._set_coords_local(True)
        self.net.build_mesh(self._active_pointmap())
        self._set_interp(True)
        pass

    def _set_interp(self, interp=None):
        self.interpolate = interp

    def _get_limits(self):
        limits = self.limits[:]  # Copy the Limits
        if self._coords_local:
            limits[0] = self.tool_limits[0]
            limits[1] = self.tool_limits[1]
        return limits

    def _transform_from_local(self, coords):  # type: (LobKinematics, [float]) -> [float]
        if self._coords_local:
            coords[0] += self.tool_origins[self.active_tool][0]
            coords[1] += self.tool_origins[self.active_tool][1]

        return coords

    def _transform_to_local(self, coords):  # type: (LobKinematics, [float]) -> [float]
        if self._coords_local:
            coords[0] -= self.tool_origins[self.active_tool][0]
            coords[1] -= self.tool_origins[self.active_tool][1]

        return coords

    def _set_coords_local(self, local):  # type: (LobKinematics, bool) -> bool
        last = self._coords_local

        if last == local:
            return local

        self.toolhead.wait_moves()  # We are changing coordinate systems. Let's wait for everything to finish before fucking with things
        self.toolhead.flush_step_generation()  # We are changing coordinate systemd. Better make sure the step queue is flushed before fucking with things

        pos_local = pos_global = self.toolhead.get_position()
        if last:  # Transform to global coords
            pos_global = self._transform_from_local(pos_local)
            self._coords_local = local
            self.toolhead.set_position(pos_global)
        else:  # Transform to local coords
            self._coords_local = local
            pos_local = self._transform_to_local(pos_global)
            self.toolhead.set_position(pos_local)

        log("Coordinate change %s. Local: %s Global: %s" % (
        "Global-->Local" if local else "Local-->Global", pos_local, pos_global), 'gr')
        return last

    def _switch_tool(self, tool):  # type: (LobKinematics, int) -> None
        self._check_shutdown()
        log("Tool Change to %s" % tool)
        self._manual_home([2])  # Move old toolhead out of the way
        self._stop_all_spindles()  # Disable any running spindles
        self._set_tool(tool)
        self._manual_home([2])  # Home the new toolhead

        # Enable Local Coordinates and move to tool origin
        # self._set_coords_local(True)
        # self.manual_move([0, 0, None, None], self.toolhead.max_velocity)

    def _set_tool(self, tool):  # type: (LobKinematics, int) -> None
        if self.active_tool == tool:
            return

        last_coords_state = self._set_coords_local(False)

        # self.toolhead.wait_moves()  # wait for all moves to finish before fucking with the tools
        self.toolhead.flush_step_generation()  # flush steps

        # Deactivate previous tool rails, caching position and limits
        for axis, name in enumerate(self.tool_configs[self.active_tool]):
            if name not in self.tool_configs[tool]:
                self.rails[name]['limits'] = tuple(self.limits[axis])
                self.rails[name]['pos'] = self.rails[name]['rail'].get_commanded_position()
                self.rails[name]['rail'].set_trapq(None)  # Remove solver from inactive rail

        # Get the homing object
        self.probe.set_active_rail(None)  # Disable probe

        # Activate new tool rails, retrieving cached position and limits
        pos = self.toolhead.get_position()
        swapped_steppers = []
        for axis, name in enumerate(self.tool_configs[tool]):
            if name not in self.tool_configs[self.active_tool]:
                self.limits[axis] = tuple(self.rails[name]['limits'])
                if self.limits[axis][0] < self.limits[axis][1]: #This axis has been homed
                    pos[axis] = self.rails[name]['pos']
                else:
                    pos[axis] = self.rails[name]['rail'].get_commanded_position()
                swapped_steppers.append(self.rails[name]['rail'])

                if name in self.probe_rails:
                    log("Setting rail %s as active"%name, '{r}')
                    self.probe.set_active_rail(name)

        for rail in swapped_steppers:
            rail.set_trapq(self.toolhead.get_trapq())  # Add solver to new rail
            rail.set_position(pos)

        self.toolhead.set_position(pos)
        self.active_tool = tool

        self._set_coords_local(last_coords_state)

    def _set_spindle_speed(self, spindle, rpm):
        self._check_shutdown()
        print_time = self.toolhead.get_last_move_time()
        if spindle < len(self.spindles):
            self.spindles[spindle].set_rpm(print_time, rpm)
            self.spindle_on = rpm > 0

    def _stop_all_spindles(self):
        print_time = self.toolhead.get_last_move_time()
        for spindle in self.spindles:
            spindle.set_rpm(print_time, 0)
        self.spindle_on = False

    def _manual_home(self, axes):  # type: (LobKinematics, [int]) -> None
        self._check_shutdown()
        last_coords_state = self._set_coords_local(False)  # Coordinate Transformation

        homing_state = Homing(self.printer)
        homing_state.set_axes(axes)
        try:
            self.home(homing_state)
        except self.printer.command_error:
            if self.printer.is_shutdown():
                raise self.printer.command_error(
                    "Homing failed due to printer shutdown")
            self.printer.lookup_object('stepper_enable').motor_off()
            raise

        self._set_coords_local(last_coords_state)

    def _home_all(self):  # type: (LobKinematics) -> None
        self._check_shutdown()
        last_coords_state = self._set_coords_local(False)  # Coordinate Transformation

        for tool in range(3):
            self._set_tool(tool)
            self._manual_home([2])
        self._set_tool(3)
        self._manual_home([0, 2])
        self._set_tool(0)
        # self._manual_home([0, 1, 2])
        self._flip_pcb(0, True)

        self._set_coords_local(last_coords_state)  # Transform back

    def _flip_pcb(self, side, homing=False):  # type: (LobKinematics, int, bool) -> None
        self._check_shutdown()
        # Skip flipping if invalid or already flipped
        if side < 0 or side > 1:
            logging.error("Invalid PCB Side, specify 1 or 0")
        elif self.pcb_side == side:
            return

        log("Flipping PCB to Side %s with homing=%s" % (side, homing), 'gr')
        last_coords_state = self._set_coords_local(False)  # Set to absolute coords

        # Home all axes which need it
        homing_axes = []
        for axis, rail in enumerate(self.get_active_rails()):  # type: int, stepper.PrinterRail
            if self.limits[axis][0] > self.limits[axis][1]:
                homing_axes.append(axis)
                continue
        self._manual_home(homing_axes)

        # Get initial pos, set to endstop position if exceeding bounds
        initial_pos = self.toolhead.get_position()[:3]
        initial_pos = numpy.clip(initial_pos, [s[0] for s in self.limits],
                                 [s[1] for s in self.limits]).tolist()

        # Move to flipping position
        flip_pos = [rail.position_endstop for rail in self.get_active_rails()]
        self.manual_move([None, None, flip_pos[2], None], self.toolhead.max_velocity) # Move Z first to avoid crashes
        self.manual_move(flip_pos[:2] + [None, None], self.toolhead.max_velocity) #Then move XY

        # Try flipping 3 times before giving up
        flip_dist = 200
        step_dist = self.pcb_stepper.steppers[0].get_step_dist()
        for _ in range(3):  # Try 3 times then give up
            start_pos = self.pcb_stepper.steppers[0].get_mcu_position()
            self.pcb_stepper.do_set_position(flip_dist * (1 - side))
            self.pcb_stepper.do_homing_move(flip_dist * side, 180, 0, True, False)
            end_pos = self.pcb_stepper.steppers[0].get_mcu_position()

            flip_dist -= abs((start_pos - end_pos) * step_dist)
            if homing or flip_dist <= 5:
                break
            log("CRASH DETECTED: %s distance to go" % flip_dist, 'o')

        if not homing and flip_dist > 0:
            log("Unable to complete flip", 'r')

        self.pcb_side = side

        self.manual_move(initial_pos[:2] + [None, None], self.toolhead.max_velocity)  # Move xy first to avoid crash
        self.manual_move([None, None, initial_pos[2], None], self.toolhead.max_velocity)  # Then move z

        self._set_coords_local(last_coords_state)

    # =================================================
    # Command Handlers
    # =================================================
    # def estop_handler(self):

    # PCB flip
    def cmd_INIT_LOB(self, gcmd):  # type: (LobKinematics, gcode.GCodeCommand) -> None
        self._check_shutdown()
        self._home_all()
        self._set_coords_local(True)
        self._set_tool(0)
        self.manual_move([0, 0, None, None], self.toolhead.get_max_velocity())
        self._build_mesh()
        # self._set_interp(True)

    def cmd_M42069(self, gcmd):  # type: (LobKinematics, gcode.GCodeCommand) -> None
        self._check_shutdown()
        side = gcmd.get_int("S", None, minval=0, maxval=1)
        if side is None:
            gcmd.error("Must specify a side to flip to")
            return
        elif self.active_tool > 2:
            gcmd.error("Switch to spindle to flip")

        self._flip_pcb(side)
        gcmd.respond_info("PCB flipped to %s" % side)

    # "Toolchange" (realy just switches which axes are active)
    def cmd_M06(self, gcmd):  # type: (LobKinematics, gcode.GCodeCommand) -> None
        self._check_shutdown()
        tool = gcmd.get_int("T", None, minval=0, maxval=len(self.tool_configs) - 1)

        if tool is None:  # Error if tool isn't specified
            gcmd.error("Must specify a tool to change to")
            return
        elif tool == self.active_tool:  # Do nothing if this tool is already selected
            gcmd.respond_info("'%s' is already active" % (self.tool_names[tool]))
            return

        self._switch_tool(tool)
        gcmd.respond_info("Tool Change Completed. %s is now active" % (self.tool_names[tool]), log=False)

    # Spindle on Clockwise
    def cmd_M03(self, gcmd):  # type: (LobKinematics, gcode.GCodeCommand) -> None
        self._check_shutdown()
        self._set_spindle_speed(self.active_tool, self.spindle_speed)

    # Spindle on CCW
    def cmd_M04(self, gcmd):  # type: (LobKinematics, gcode.GCodeCommand) -> None
        self._check_shutdown()
        self._set_spindle_speed(self.active_tool, self.spindle_speed)

    # Spindle off
    def cmd_M05(self, gcmd):  # type: (LobKinematics, gcode.GCodeCommand) -> None
        self._check_shutdown()
        self._stop_all_spindles()

    # Spindle Speed
    def cmd_S(self, gcmd):  # type: (LobKinematics, gcode.GCodeCommand) -> None
        self._check_shutdown()
        try:
            speed = int(gcmd.get_command_parameters()["S"])
            self.spindle_speed = speed
            gcmd.respond_info("Spindle Speed set to %d RPM" % speed)
            if self.active_tool < len(self.spindles) and self.spindle_on:
                self._set_spindle_speed(self.active_tool, self.spindle_speed)
        except:
            gcmd.error("Invalid Spindle Speed: %s" % gcmd.get_command_parameters()["S"])

    def cmd_F(self, gcmd):
        self._check_shutdown()
        log(gcmd.get_command_parameters(), "{g}")
        feed = int(gcmd.get_command_parameters()["F"])
        self.feed_rate = feed

def load_kinematics(toolhead, config):
    return LobKinematics(toolhead, config)
