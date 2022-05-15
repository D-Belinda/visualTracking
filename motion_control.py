import numpy as np

FRAME_WIDTH = 960
FRAME_HEIGHT = 720

Kx = np.array([1.0, 0.2, -2.5])  # P, I, D constants
Ky = np.array([1, 1, 1])  # P, I, D constants
Ksize = np.array([1, 1, 1])  # P, I, D constants

MAX_SPEED = 100

FADE_COEFFICIENT = 1 / 3


class motion_controller:
    PIX_TO_DIST: float

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
            dxys = np.add(dxys*FADE_COEFFICIENT, np.subtract(circle_np, xys)/self.FPS) / (1+FADE_COEFFICIENT)

        xys = np.add(FADE_COEFFICIENT*xys, circle_np) / (1+FADE_COEFFICIENT)

        ixys = np.add(ixys, circle_np)
        ixys = np.clip(ixys, -500, 500)

        self.__update_params_tuple(xys, dxys, ixys)

    def instruct(self, diagnostic=False):
        dx_drone = float(np.sum(np.multiply(Kx, np.array([self.x, self.dx, self.ix]))))
        dy_drone = float(np.sum(np.multiply(Ky, np.array([self.y, self.dy, self.iy]))))
        dz_drone = float(np.sum(np.multiply(Ksize, np.array([self.size, self.dsize, self.isize]))))

        if diagnostic:
            print(self.x, self.dx, self.ix)
        ret = np.array([dx_drone, 0, 0])*self.PIX_TO_DIST*self.INSTRUCTION_INTERVAL
        ret = np.clip(ret, -MAX_SPEED, MAX_SPEED)
        return tuple(ret)

    def get_obj_displacement(self):
        return self.x, self.y, self.size

    def get_obj_velocity(self):
        return self.dx, self.dy, self.dsize

    def get_obj_integral(self):
        return self.ix, self.iy, self.isize
