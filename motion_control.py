"""
PURPOSE: given the object's relative location in the x(horizontal), y(vertical), and z(distance), return the desired
velocity of the drone in these three dimensions
"""

import numpy as np
import math

FRAME_WIDTH = 960
FRAME_HEIGHT = 720

Kx = np.array([0.2, 0.03, 0.1]) * 1  # P, I, D constants, 1/0 is on/off switch
Ky = np.array([1.0, 0.05, 0.13]) * 1
Kz = np.array([0.4, 0.02, 0.12]) * 1
Kr = np.array([0.7, 0.07, 0.02]) * 1

MAX_SPEED = 70  # max speed of the drone that will be assigned

FADE_COEFFICIENT = 1 / 3  # the previous frame is weighted 1/3 of the current

TRACK_DIST = 100  # in cm
ITEM_SIZE = 25
DIST_TO_PIX_AT_90 = 12.5/120
HORIZONTAL_FOV = 62.27
VERTICAL_FOV = 75


def calc_distance(item_size, n_pixels):
    return item_size / np.arctan(math.radians(n_pixels / 720 * VERTICAL_FOV))


def calc_error(distance, n_pixels):
    angle = n_pixels / 960 * HORIZONTAL_FOV
    return np.tan(math.radians(angle)) * distance


class MotionController:
    def clear_data(self):
        self.x = self.y = self.z = self.r = 0.0
        self.dx = self.dy = self.dz = self.dr = 0.0
        self.ix = self.iy = self.iz = self.ir = 0.0

    def __init__(self, fps):
        self.x = self.y = self.z = self.r = 0.0
        self.dx = self.dy = self.dz = self.dr = 0.0
        self.ix = self.iy = self.iz = self.ir = 0.0
        self.FPS = fps
        # self.DIST_TO_PIX = 1 / 12  # the ratio between distance (cm) and the number of pixels
        self.I_MAX = 25  # maximum value for the integral component to prevent blow-ups
        self.cur_distance = TRACK_DIST

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

        rect[0] -= FRAME_WIDTH/2
        rect[1] -= FRAME_HEIGHT/2

        # rect_np is the numpy array of the object's location in the current frame
        wh = rect[2]
        self.cur_distance = calc_distance(ITEM_SIZE, wh[1])
        rect[0] = calc_error(self.cur_distance, rect[0])
        rect[1] = calc_error(self.cur_distance, rect[1])
        rect[2] = self.cur_distance - TRACK_DIST

        # calculate angle of rotation
        tilt_dir = rect[3]
        angle = tilt_dir * math.degrees(np.arccos((wh[0]/wh[1]) / 1.5))
        angle = 0 if math.isnan(angle) else angle
        rect[3] = angle
        print(wh, tilt_dir, angle)

        rect_np = np.array(rect)

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

        return [int(e) for e in rect]

    # returns the desired speed in the form of a tuple (x, y, z)
    def instruct(self, diagnostic=False):
        dx_drone = float(np.sum(np.multiply(Kx, np.array([self.x, self.ix, self.dx]))))

        dy_drone = float(np.sum(np.multiply(Ky, np.array([self.y, self.iy, self.dy]))))
        dz_drone = float(np.sum(np.multiply(Kz, np.array([self.z, self.iz, self.dz]))))
        dr_drone = float(np.sum(np.multiply(Kr, np.array([self.r, self.ir, self.dr]))))

        d_tangent = .65 * math.radians(dr_drone)*self.cur_distance
        print(d_tangent, dx_drone)
        dx_drone += d_tangent

        if diagnostic:
            print('PID', self.x, self.ix, self.dx)
            print(list(np.multiply(Kx, np.array([self.x, self.ix, self.dx]))))
        # dr_drone is in degrees, should not be affected by DIST_TO_PIX
        ret = np.array([dx_drone, -dy_drone, dz_drone, -dr_drone])
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
