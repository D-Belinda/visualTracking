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

    def update_obj(self, d: tuple, v: tuple, a: tuple):
        self.obj_d.append(d)
        self.obj_v.append(v)
        self.obj_a.append(a)

    def graph_drone(self):
        pass

    def graph_obj(self):
        pass
