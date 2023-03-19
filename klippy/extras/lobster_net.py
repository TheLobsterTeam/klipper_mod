import logging, math, json, collections
from . import probe
import configfile
from .lobster_log import log, trace, signpost


class LobsterNet:
    def __init__(self, config):  # type: (LobsterNet, configfile.ConfigWrapper) -> None
        # Lobster net Parameters
        self.x_range = sorted(config.getfloatlist("x_range"))
        self.y_range = sorted(config.getfloatlist("y_range"))
        self.res_x = config.getint("resolution_x", minval=2)
        self.res_y = config.getint("resolution_y", minval=2)

        self.interp_dist = config.getfloat("interp_dist", minval=0.1)

        self.meshpoints = {'x': [[self.x_range[0] for _ in range(self.res_y)] for _ in range(self.res_x)],
                           'y': [[self.y_range[0] for _ in range(self.res_y)] for _ in range(self.res_x)]}

        dist_x = (self.x_range[1] - self.x_range[0]) / (self.res_x - 1)
        dist_y = (self.y_range[1] - self.y_range[0]) / (self.res_y - 1)
        for x in range(self.res_x):
            for y in range(self.res_y):
                self.meshpoints['x'][x][y] += dist_x * x
                self.meshpoints['y'][x][y] += dist_y * y

        self.mesh_map = {}
        self.printer = config.get_printer()
        self.probe = self.printer.lookup_object('lobster_probe')

    def get_empty_pointmap(self):
        pointmap = {'x': self.meshpoints['x'],
                    'y': self.meshpoints['y'],
                    'z': [[0. for _ in range(self.res_y)] for _ in range(self.res_x)]}
        return pointmap, self.res_x, self.res_y

    def register_pointmap(self, name, pointmap):
        self.mesh_map[name] = pointmap['z']

    def get_interpolated(self, name, pos):
        [x, y] = pos[:2]
        x_out_of_bounds = x < self.x_range[0] or x > self.x_range[1]
        y_out_of_bounds = y < self.y_range[0] or y > self.y_range[1]
        if x_out_of_bounds or y_out_of_bounds:
            return 0
        elif name not in self.mesh_map.keys():
            return 0

        z_mesh = self.mesh_map[name]
        return self._interp(pos, z_mesh)

    def _interp(self, pos, z_mesh):
        [x, y] = pos[:2]

        step_x = (self.x_range[1] - self.x_range[0]) / (self.res_x - 1)
        x_dist = (x - self.x_range[0])
        x_idx = int(x_dist / step_x)
        x_idx = self.res_x - 2 if x_idx >= self.res_x - 1 else x_idx
        x_scale = (x_dist - x_idx * step_x) / step_x

        step_y = (self.y_range[1] - self.y_range[0]) / (self.res_y - 1)
        y_dist = (y - self.y_range[0])
        y_idx = int(y_dist / step_y)
        y_idx = self.res_y - 2 if y_idx >= self.res_y - 1 else y_idx
        y_scale = (y_dist - y_idx * step_y) / step_y

        p00 = z_mesh[x_idx][y_idx] * (1 - x_scale) * (1 - y_scale)
        p01 = z_mesh[x_idx][y_idx + 1] * (1 - x_scale) * (y_scale)
        p10 = z_mesh[x_idx + 1][y_idx] * (x_scale) * (1 - y_scale)
        p11 = z_mesh[x_idx + 1][y_idx + 1] * (x_scale) * (y_scale)
        return p00 + p01 + p10 + p11

    def build_mesh(self, name):
        toolhead = self.printer.lookup_object("toolhead")  # type: toolhead.ToolHead
        signpost("BUILDING MESH FOR %s" % name, 'p')

        toolhead.manual_move([None, None, self.probe.safe_height()], toolhead.max_velocity)

        xmap = self.meshpoints['x']
        ymap = self.meshpoints['y']
        zmap = [[0. for _ in range(self.res_y)] for _ in range(self.res_x)]

        for x in range(self.res_x):
            y_range = range(self.res_y) if x % 2 == 0 else reversed(range(self.res_y))
            for y in y_range:
                toolhead.manual_move([xmap[x][y], ymap[x][y]], toolhead.max_velocity)
                zmap[x][y] = self.probe.probe_point(1)  # TODO: Remove manual sample size

        toolhead.manual_move([None, None, self.probe.safe_height()], toolhead.max_velocity)
        self.mesh_map[name] = zmap
        pass

    def interp_path(self, name, start_pos, end_pos):
        [start_x, start_y, start_z] = start_pos[:3]
        [end_x, end_y, end_z] = end_pos[:3]
        log("Interpolating from %s to %s"%(start_pos, end_pos), 'r')

        x_out_of_bounds = min(start_x, end_x) < self.x_range[0] or max(start_x, end_x) > self.x_range[1]
        y_out_of_bounds = min(start_y, end_y) < self.y_range[0] or max(start_y, end_y) > self.y_range[1]
        if x_out_of_bounds or y_out_of_bounds:
            return []
        elif name not in self.mesh_map.keys():
            return []

        delta_x = end_x - start_x
        delta_y = end_y - start_y
        delta_z = end_z - start_z
        path_len = math.sqrt(delta_x*delta_x + delta_y*delta_y)
        interp_points = int(max(1, math.ceil(path_len/self.interp_dist)))
        delta_x /= interp_points
        delta_y /= interp_points
        delta_z /= interp_points

        z_mesh = self.mesh_map[name]
        path = []
        for i in range(1, interp_points + 1):
            x = start_x + delta_x*i
            y = start_y + delta_y*i
            z = start_z + delta_z*i + self._interp([x, y], z_mesh)
            path.append([x, y, z])
        return path


def load_config(config):
    return LobsterNet(config)
