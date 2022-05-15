from collections import deque

FRAME_WIDTH = 960
FRAME_HEIGHT = 720

Kx = (1.0, 0.2, -2.5)  # P, I, D constants
Ky = (1, 1, 1)  # P, I, D constants
Ksize = (1, 1, 1)  # P, I, D constants

MAX_SPEED = 100


def fade_function(n: int):
    # to decrease the weight of older frames w a geometric series that decreases over time as coefficients
    return (1 / 4) ** n


def ff_sum(lower_bound: int, upper_bound: int):  # calculate the sum of the series in [lower, upper)
    constant = fade_function(1)
    ret = (fade_function(lower_bound)-fade_function(upper_bound)) / (1 - constant)
    return max(1.0, ret)


class motion_controller:
    def __init__(self, fps, instruction_interval):
        self.x = self.y = self.size = self.dx = self.dy = self.dsize = 0.0
        self.ix = self.iy = self.isize = 0.0
        self.q = deque()  # a queue of circles
        self.FPS = fps
        self.PIX_TO_DIST = 1 / 10  # adjust based on distance, fix later
        self.LOCATION_DELAY = 0.1  # use the average of the datas from the last 0.1 seconds to determine the location and size of the object
        self.VELOCITY_DELAY = 0.1
        self.MAX_QUEUE_LENGTH = int(max(self.LOCATION_DELAY, self.VELOCITY_DELAY) * self.FPS + 1)
        self.INSTRUCTION_INTERVAL = instruction_interval

    def __update_params(self, x, y, size, dx, dy, dsize):
        self.x = x
        self.y = y
        self.size = size
        self.dx = dx
        self.dy = dy
        self.dsize = dsize

    def add_location(self, circle):
        if len(circle) == 0:
            self.q = deque()
        else:
            self.q.appendleft(circle)
        while len(self.q) > self.MAX_QUEUE_LENGTH:
            self.q.pop()

    def __process(self):
        n_frames = min(len(self.q), int(self.LOCATION_DELAY * self.FPS))
        if n_frames == 0:
            return
        x, y, size = 0, 0, 0
        for i in range(n_frames):
            x += self.q[i][0]*fade_function(i)
            y += self.q[i][1]*fade_function(i)
            size += self.q[i][2]*fade_function(i)
        x /= ff_sum(0, n_frames)
        y /= ff_sum(0, n_frames)
        size /= ff_sum(0, n_frames)

        n_frames = min(len(self.q), int(self.VELOCITY_DELAY * self.FPS))
        if n_frames == 0:
            return
        dx = dy = dsize = 0
        for i in range(1, n_frames):
            dx += (self.q[i-1][0] - self.q[i][0]) / self.FPS * fade_function(i)
            dy += (self.q[i-1][1] - self.q[i][1]) / self.FPS * fade_function(i)
            dsize += (self.q[i-1][2] - self.q[i][2]) / self.FPS * fade_function(i)
        dx /= ff_sum(1, n_frames)
        dy /= ff_sum(1, n_frames)
        dsize /= ff_sum(1, n_frames)

        self.ix += x * self.INSTRUCTION_INTERVAL / self.FPS
        self.iy += y * self.INSTRUCTION_INTERVAL / self.FPS
        self.isize += size * self.INSTRUCTION_INTERVAL / self.FPS
        self.ix = min(max(self.ix, -500), 500)
        self.iy = min(max(self.iy, -500), 500)
        self.isize = min(max(self.isize, -500), 500)

        self.__update_params(x, y, size, dx, dy, dsize)

    def instruct(self, diagnostic=True):
        self.__process()
        dx_drone = Kx[0] * self.x + Kx[1] * self.ix + Kx[2] * self.dx
        dx_drone *= self.PIX_TO_DIST
        dx_drone = min(max(dx_drone, -MAX_SPEED), MAX_SPEED)

        if diagnostic:
            print(self.x, self.dx, self.ix)
        return (dx_drone, 0, 0)

    def get_obj_displacement(self):
        return self.x, self.y, self.size

    def get_obj_velocity(self):
        return self.dx, self.dy, self.dsize

    def get_obj_integral(self):
        return self.ix, self.iy, self.isize
