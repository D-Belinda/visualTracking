# import matplotlib.pyplot as plt
MAX_SIZE = 1000


class logger:
    def __init__(self):
        self.drone_d = self.drone_v = self.drone_i = []
        self.obj_d = self.obj_v = self.obj_i = []

    def update_drone(self, d: tuple, v: tuple, i: tuple):
        self.drone_d.append(d)
        self.drone_v.append(v)
        self.drone_i.append(i)
        if len(self.drone_d) > MAX_SIZE:
            self.drone_d.pop(0)
        if len(self.drone_v) > MAX_SIZE:
            self.drone_v.pop(0)
        if len(self.drone_i) > MAX_SIZE:
            self.drone_d.pop(0)

    def update_obj(self, d: tuple, v: tuple, a: tuple):
        self.obj_d.append(d)
        self.obj_v.append(v)
        self.obj_i.append(a)
        if len(self.obj_d) > MAX_SIZE:
            self.obj_d.pop(0)
        if len(self.obj_v) > MAX_SIZE:
            self.obj_v.pop(0)
        if len(self.obj_i) > MAX_SIZE:
            self.obj_i.pop(0)

    def graph_drone(self):
        pass

    def graph_obj(self):
        pass
