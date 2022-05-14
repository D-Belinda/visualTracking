import numpy as np

FRAME_WIDTH = 960
FRAME_HEIGHT = 720

Kx = (1.0, 0.0, 0.4)  # P, I, D constants
Ky = (1, 1, 1)  # P, I, D constants
Ksize = (1, 1, 1)  # P, I, D constants

MAX_SPEED = 80

FADE_COEFFICIENT = 1 / 2


class motion_controller:
    def __init__(self, fps, instruction_interval):
        self.x = self.y = self.size = 0.0
        self.dx = self.dy = self.dsize = 0.0
        self.ix = self.iy = self.isize = 0.0
        self.FPS = fps
        self.PIX_TO_DIST = 1 / 20  # adjust based on distance, fix later
        self.INSTRUCTION_INTERVAL = instruction_interval

    def __update_params(self, x, y, size, dx, dy, dsize, ix, iy, isize):
        self.x = x
        self.y = y
        self.size = size
        self.dx = dx
        self.dy = dy
        self.dsize = dsize
        self.ix = ix
        self.iy = iy
        self.isize = isize

    def __update_params_tuple(self, xys, dxys, ixys):
        self.x, self.y, self.size = tuple(xys)
        self.dx, self.dy, self.dsize = tuple(dxys)
        self.ix, self.iy, self.isize = tuple(ixys)

    def add_location(self, circle):
        if len(circle) == 0: pass
        circle_np = np.array(circle)
        xys = np.array([self.x, self.y, self.size])
        dxys = np.array([self.dx, self.dy, self.dsize])
        ixys = np.array([self.ix, self.iy, self.isize])

        if not self.x == self.y == self.size == 0.0:
            dxys = np.add(dxys*FADE_COEFFICIENT, np.subtract(xys, circle_np)/self.FPS) / (1+FADE_COEFFICIENT)

        xys = np.add(FADE_COEFFICIENT*xys, circle_np) / (1+FADE_COEFFICIENT)

        ixys = np.add(ixys, circle_np)
        ixys = np.clip(ixys, -500, 500)

        self.__update_params_tuple(xys, dxys, ixys)

    def instruct(self, diagnostic=False):
        dx_drone = Kx[0] * self.x + Kx[1] * self.ix + Kx[2] * self.dx
        dx_drone *= self.PIX_TO_DIST
        dx_drone *= self.INSTRUCTION_INTERVAL
        dx_drone = min(max(dx_drone, -MAX_SPEED), MAX_SPEED)

        if diagnostic:
            print(self.x, self.dx, self.ix)
        return dx_drone, 0, 0

    def get_obj_displacement(self):
        return self.x, self.y, self.size

    def get_obj_velocity(self):
        return self.dx, self.dy, self.dsize

    def get_obj_integral(self):
        return self.ix, self.iy, self.isize
