from collections import deque

FRAME_WIDTH = 960
FRAME_HEIGHT = 720

class motion_controller:
    def __init__(self, fps):
        self.x = self.y = self.size = self.dx = self.dy = self.dsize = 0.0
        self.ddx = self.ddy = self.ddsize = 0.0
        self.q = deque()  # a queue of circles
        self.FPS = fps
        self.TIME_TO_TARGET = 1.0
        self.PIX_TO_DIST = 1/100
        self.LOCATION_DELAY = 0.1  # use the average of the datas from the last 0.1 seconds to determine the location and size of the object
        self.VELOCITY_DELAY = 0.15
        self.ACC_DELAY = 0.2
        self.MAX_QUEUE_LENGTH = int(max(self.LOCATION_DELAY, self.VELOCITY_DELAY, self.ACC_DELAY) * self.FPS + 1)

    def __update_params(self, x, y, size, dx, dy, dsize, ddx, ddy, ddsize):
        self.x = x
        self.y = y
        self.size = size
        self.dx = dx
        self.dy = dy
        self.dsize = dsize
        self.ddx = ddx
        self.ddy = ddy
        self.ddsize = ddsize

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
        dx = (self.q[n_frames - 1][0] - self.q[0][0]) / n_frames / self.FPS
        dy = (self.q[n_frames - 1][1] - self.q[0][1]) / n_frames / self.FPS
        dsize = (self.q[n_frames - 1][2] - self.q[0][2]) / n_frames / self.FPS

        n_frames = min(len(self.q), int(self.ACC_DELAY * self.FPS))
        if n_frames < 2:
            return
        vxi = (self.q[n_frames-2][0] - self.q[n_frames-1][0]) * self.FPS
        vxf = (self.q[0][0] - self.q[1][0]) * self.FPS
        ddx = (vxf-vxi) * self.FPS
        vyi = (self.q[n_frames - 2][1] - self.q[n_frames - 1][1]) * self.FPS
        vyf = (self.q[0][1] - self.q[1][1]) * self.FPS
        ddy = (vyf - vyi) * self.FPS
        vsi = (self.q[n_frames - 2][2] - self.q[n_frames - 1][2]) * self.FPS
        vsf = (self.q[0][2] - self.q[1][2]) * self.FPS
        ddsize = (vsf - vsi) * self.FPS

        self.__update_params(x,y,size,dx,dy,dsize,ddx,ddy,ddsize)

    def instruct(self):
        self.__process()
        ddx_drone = 2*self.x/(self.TIME_TO_TARGET**2) + 2*self.dx/self.TIME_TO_TARGET + self.ddx
        ddx_drone *= self.PIX_TO_DIST
        return (ddx_drone, 0, 0)
