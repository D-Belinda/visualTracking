"""
PURPOSE: given the object's relative location in the x(horizontal), y(vertical), and z(distance), return the desired
velocity of the drone in these three dimensions
"""

import numpy as np
import math


def cot(x):
    return 1/math.tan(x)


FRAME_WIDTH = 960
FRAME_HEIGHT = 720

Kx = np.array([0.7, 0.5, 0.20]) * 0  # P, I, D constants, 1/0 is on/off switch
Ky = np.array([2.0, 0.5, 0.25]) * 1
Kz = np.array([1.0, 0.5, 0.2]) * 1
Kr = np.array([0.25, 0.1, 0.05]) * 1

MAX_SPEED = 70  # max speed of the drone that will be assigned

FADE_COEFFICIENT = 1 / 3  # the previous frame is weighted 1/3 of the current


class MotionController:
    def clear_data(self):
        self.x = self.y = self.z, self.r = 0.0
        self.dx = self.dy = self.dz, self.dr = 0.0
        self.ix = self.iy = self.iz, self.ir = 0.0

    def __init__(self, fps):
        self.max_WoverH = 1
        self.x = self.y = self.z = self.r = 0.0
        self.dx = self.dy = self.dz = self.dr = 0.0
        self.ix = self.iy = self.iz = self.ir = 0.0
        self.FPS = fps
        self.DIST_TO_PIX = 1 / 12  # the ratio between distance (cm) and the number of pixels
        self.I_MAX = 25 / self.DIST_TO_PIX / Kx[1]  # maximum value for the integral component to prevent blow-ups

    def __update_params(self, x, y, z, r, dx, dy, dz, dr, ix, iy, iz, ir):
        self.x = x
        self.y = y
        self.z = z
        self.r = r
        self.dx = dx
        self.dy = dy
        self.dz = dz
        self.dr = dr
        self.ix = ix
        self.iy = iy
        self.iz = iz
        self.ir = ir

    def __update_params_tuple(self, xyz, ixyz, dxyz):
        self.x, self.y, self.z, self.r = tuple(xyz)
        self.dx, self.dy, self.dz, self.dr = tuple(dxyz)
        self.ix, self.iy, self.iz, self.ir = tuple(ixyz)

    def add_location(self, rect):
        if rect is None or len(rect) == 0:
            return

        # rect_np is the numpy array of the object's location in the current frame
        wh = rect[2]
        # angle is the field of view in radian * proportion of the object on screen
        # the use of cotangent is explained more in the email I sent to Dr Fu in the beginning of the summer
        rect[2] = wh[1] / np.sqrt(3)
        rect[2] *= cot(math.radians(82.6) * rect[2] / 960) - cot(math.radians(10))
        # calculate angle of rotation
        tilt_dir = rect[3]
        angle = tilt_dir * math.degrees(np.arccos((wh[0]/wh[1]) / 1.45))
        angle = 0 if math.isnan(angle) else angle
        rect[3] = angle
        print(wh, tilt_dir, angle)

        rect_np = np.array(rect)
        rect_np[0] = rect_np[0] - FRAME_WIDTH / 2
        rect_np[1] = rect_np[1] - FRAME_HEIGHT / 2

        # the xyzr's are first set to the location of the object in the previous frame
        xyzr = np.array([self.x, self.y, self.z, self.r])
        dxyzr = np.array([self.dx, self.dy, self.dz, self.dr])
        ixyzr = np.array([self.ix, self.iy, self.iz, self.ir])

        # merges the locations from the previous frame with the current
        if not self.x == self.y == self.z == self.r == 0.0:
            dxyzr = (FADE_COEFFICIENT*dxyzr + (rect_np - xyzr)*self.FPS) / (1+FADE_COEFFICIENT)

        xyzr = (FADE_COEFFICIENT * xyzr + rect_np) / (1 + FADE_COEFFICIENT)

        ixyzr = np.add(ixyzr, rect_np / self.FPS)
        ixyzr = np.clip(ixyzr, -self.I_MAX, self.I_MAX)

        self.__update_params_tuple(xyzr, ixyzr, dxyzr)

    # returns the desired speed in the form of a tuple (x, y, z)
    def instruct(self, diagnostic=False):
        dx_drone = float(np.sum(np.multiply(Kx, np.array([self.x, self.ix, self.dx]))))
        dy_drone = float(np.sum(np.multiply(Ky, np.array([self.y, self.iy, self.dy]))))
        dz_drone = float(np.sum(np.multiply(Kz, np.array([self.z, self.iz, self.dz]))))
        dr_drone = float(np.sum(np.multiply(Kr, np.array([self.r, self.ir, self.dr]))))

        if diagnostic:
            print('PID', self.x, self.ix, self.dx)
            print(list(np.multiply(Kx, np.array([self.x, self.ix, self.dx]))))
        # dr_drone is in degrees, should not be affected by DIST_TO_PIX
        ret = np.array([dx_drone, -dy_drone, dz_drone, dr_drone/self.DIST_TO_PIX]) * self.DIST_TO_PIX
        ret = np.clip(ret, -MAX_SPEED, MAX_SPEED)
        return ret.astype(int)

    def get_obj_displacement(self):
        return np.array([self.x, self.y, self.z])

    def get_obj_velocity(self):
        return np.array([self.dx, self.dy, self.dz])

    def get_obj_integral(self):
        return np.array([self.ix, self.iy, self.iz])

    def get_x_info(self):
        return np.array([self.x, self.ix, self.dx])

    def get_y_info(self):
        return np.array([self.y, self.iy, self.dy])

    def get_z_info(self):
        return np.array([self.z, self.iz, self.dz])
