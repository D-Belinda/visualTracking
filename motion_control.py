from collections import deque

FRAME_WIDTH = 960
FRAME_HEIGHT = 720

Kx = (1.0, 0.0, 0.4)  # P, I, D constants
Ky = (1, 1, 1)  # P, I, D constants
Ksize = (1, 1, 1)  # P, I, D constants

MAX_SPEED = 80

class motion_controller:
    def __init__(self, fps, instruction_interval):
        self.x = self.y = self.size = self.dx = self.dy = self.dsize = 0.0
        self.ix = self.iy = self.isize = 0.0
        self.q = deque()  # a queue of circles
        self.FPS = fps
        self.TIME_TO_TARGET = 0.5  # sensitivity
        self.PIX_TO_DIST = 1 / 20  # adjust based on distance, fix later
        self.LOCATION_DELAY = 0.05  # use the average of the datas from the last 0.1 seconds to determine the location and size of the object
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
            x += self.q[i][0]
            y += self.q[i][1]
            size += self.q[i][2]
        x = x / n_frames
        y = y / n_frames
        size = size / n_frames

        n_frames = min(len(self.q), int(self.VELOCITY_DELAY * self.FPS))
        if n_frames == 0:
            return
        dx = (self.q[0][0] - self.q[n_frames - 1][0]) / (n_frames / self.FPS)
        dy = (self.q[0][1] - self.q[n_frames - 1][1]) / (n_frames / self.FPS)
        dsize = (self.q[0][2] - self.q[n_frames - 1][2]) / (n_frames / self.FPS)

        self.ix += x * self.INSTRUCTION_INTERVAL / self.FPS
        self.iy += y * self.INSTRUCTION_INTERVAL / self.FPS
        self.isize += size * self.INSTRUCTION_INTERVAL / self.FPS

        self.ix = min(max(self.ix, -500), 500)
        self.iy = min(max(self.iy, -500), 500)
        self.isize = min(max(self.isize, -500), 500)

        self.__update_params(x, y, size, dx, dy, dsize)

    def instruct(self, diagnostic=True):
        self.__process()
        '''
        ddx_drone = 2*self.x/(self.TIME_TO_TARGET**2) + 2*self.dx/self.TIME_TO_TARGET + self.ddx
        ddx_drone *= self.PIX_TO_DIST
        '''
        # dx_drone = (self.x + self.dx * self.TIME_TO_TARGET) / self.TIME_TO_TARGET
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
