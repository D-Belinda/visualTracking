import numpy as np
from math import radians, tan


def cot(x):
    return 1/tan(x)


FRAME_WIDTH = 960
FRAME_HEIGHT = 720

Kx = np.array([1.0, 0.2, -5.0]) * 1  # P, I, D constants, 1/0 is on/off switch
Ky = np.array([2.0, 0.4, -4.0]) * 1  # P, I, D constants
Kz = np.array([1.0, 0.2, -5.0]) * 1  # P, I, D constants

MAX_SPEED = 100

FADE_COEFFICIENT = 1 / 3


class motion_controller:

    def __init__(self, fps, instruction_interval):
        self.x = self.y = self.z = 0.0
        self.dx = self.dy = self.dz = 0.0
        self.ix = self.iy = self.iz = 0.0
        self.FPS = fps
        self.DIST_TO_PIX = 1 / 12  # adjust based on distance, fix later
        self.INSTRUCTION_INTERVAL = instruction_interval
        self.I_MAX = 50 / self.DIST_TO_PIX / Kx[1]  # maximum value for ix

    def __update_params(self, x, y, z, dx, dy, dz, ix, iy, iz):
        self.x = x
        self.y = y
        self.z = z
        self.dx = dx
        self.dy = dy
        self.dz = dz
        self.ix = ix
        self.iy = iy
        self.iz = iz

    def __update_params_tuple(self, xyz, ixyz, dxyz):
        self.x, self.y, self.z = tuple(xyz)
        self.dx, self.dy, self.dz = tuple(dxyz)
        self.ix, self.iy, self.iz = tuple(ixyz)

    def add_location(self, circle):
        if len(circle) == 0:
            return
        # angle = field of view in radian * proportion of the object on screen
        circle_np = np.array(circle)
        circle_np[0] = circle_np[0] - FRAME_WIDTH / 2
        circle_np[1] = circle_np[1] - FRAME_HEIGHT / 3
        circle_np[2] = circle[2] * (cot(radians(82.6) * circle[2] / 960) - cot(radians(5)))

        xyz = np.array([self.x, self.y, self.z])
        dxyz = np.array([self.dx, self.dy, self.dz])
        ixyz = np.array([self.ix, self.iy, self.iz])

        if not self.x == self.y == self.z == 0.0:
            dxyz = np.add(dxyz * FADE_COEFFICIENT, np.subtract(circle_np, xyz) / self.FPS) \
                       / (1 + FADE_COEFFICIENT)

        xyz = np.add(FADE_COEFFICIENT * xyz, circle_np) / (1 + FADE_COEFFICIENT)

        ixyz = np.add(ixyz, circle_np / self.FPS)
        ixyz = np.clip(ixyz, -self.I_MAX, self.I_MAX)

        self.__update_params_tuple(xyz, ixyz, dxyz)

    def instruct(self, diagnostic=False):
        dx_drone = float(np.sum(np.multiply(Kx, np.array([self.x, self.ix, self.dx]))))
        dy_drone = float(np.sum(np.multiply(Ky, np.array([self.y, self.iy, self.dy]))))
        dz_drone = float(np.sum(np.multiply(Kz, np.array([self.z, self.iz, self.dz]))))

        if diagnostic:
            print('PID', self.z, self.iz, self.dz)
            print(list(np.multiply(Kz, np.array([self.z, self.iz, self.dz]))))
        ret = np.array([dx_drone, -dy_drone, dz_drone]) * self.DIST_TO_PIX * self.INSTRUCTION_INTERVAL
        ret = np.clip(ret, -MAX_SPEED, MAX_SPEED)
        return tuple(ret)

    def get_obj_displacement(self):
        return self.x, self.y, self.z

    def get_obj_velocity(self):
        return self.dx, self.dy, self.dz

    def get_obj_integral(self):
        return self.ix, self.iy, self.iz
