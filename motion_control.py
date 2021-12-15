from collections import deque

FRAME_WIDTH = 960
FRAME_HEIGHT = 720

class motion_controller:
    class target:
        def __init__(self):
            self.x = self.y = self.size = self.dx = self.dy = self.dsize = None
        def __int__(self, x, y, size, dx, dy, dsize):
            self.x = x
            self.y = y
            self.size = size
            self.dx = dx
            self.dy = dy
            self.dsize = dsize

    def __init__(self, fps):
        self.q = deque()  # a queue of circles
        self.FPS = fps
        self.LOCATION_DELAY = 0.1  # use the average of the datas from the last 0.1 seconds to determine the location and size of the object
        self.MOTION_DELAY = 0.2
        self.MAX_QUEUE_LENGTH = 20
        self.target = self.target()

    def add(self, circle):
        if len(circle) == 0:
            self.q = deque()
        else:
            self.q.appendleft(circle)
        while len(self.q) > self.MAX_QUEUE_LENGTH:
            self.q.pop()

    def __process(self):
        n_frames = min(len(self.q), int(self.LOCATION_DELAY * self.FPS))
        if n_frames == 0:
            self.target = self.target()
            return
        x, y, size = 0, 0, 0
        for i in range(n_frames):
            x += self.q[i][0]
            y += self.q[i][1]
            size += self.q[i][2]
        x = x / n_frames
        y = y / n_frames
        size = size / n_frames

        n_frames = min(len(self.q), int(self.MOTION_DELAY * self.FPS))
        if n_frames == 0:
            self.target = self.target()
            return
        dx = (self.q[len(self.q) - 1][0] - self.q[0][0]) / (len(self.q) / self.FPS)
        dy = (self.q[len(self.q) - 1][1] - self.q[0][1]) / (len(self.q) / self.FPS)
        dsize = (self.q[len(self.q) - 1][2] - self.q[0][2]) / (len(self.q) / self.FPS)
        self.target = self.target(x, y, size, dx, dy, dsize)

    def instruct(self):
        events = []
        ret = self.target.x, self.target.y, self.target.size
        if ret is None:
            return events
        x, y, size = ret
        mid_x_range = FRAME_WIDTH/3, FRAME_WIDTH*2/3
        if x < mid_x_range[0]:
            events.append("left")
        elif x > mid_x_range[1]:
            events.append("right")
        # print(events)
        return events
