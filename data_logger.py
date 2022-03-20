import matplotlib.pyplot as plt
MAX_SIZE = 10000


class logger:
    def __init__(self):
        self.drone_d = self.drone_v = self.drone_a = []
        self.obj_d = self.obj_v = self.obj_a = []

    def update_drone(self, d: tuple, v: tuple, a: tuple):
        self.drone_d.append(d)
        self.drone_v.append(v)
        self.drone_a.append(a)
        if len(self.drone_d) > MAX_SIZE:
            self.drone_d.pop(0)
        if len(self.drone_v) > MAX_SIZE:
            self.drone_v.pop(0)
        if len(self.drone_a) > MAX_SIZE:
            self.drone_d.pop(0)

    def update_obj(self, d: tuple, v: tuple, a: tuple):
        self.obj_d.append(d)
        self.obj_v.append(v)
        self.obj_a.append(a)
        if len(self.obj_d) > MAX_SIZE:
            self.obj_d.pop(0)
        if len(self.obj_v) > MAX_SIZE:
            self.obj_v.pop(0)
        if len(self.obj_a) > MAX_SIZE:
            self.obj_a.pop(0)

    def graph_drone(self):
        pass

    def graph_obj(self):
        pass
