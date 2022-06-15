import matplotlib.pyplot as plt
import numpy as np
MAX_SIZE = 300


class Logger:
    def __init__(self, obj_plot=True, drone_plot=True):
        tmp = np.zeros((MAX_SIZE, 3))
        self.obj_x_info, self.obj_y_info, self.obj_z_info = tmp.copy(), tmp.copy(), tmp.copy()
        tmp = np.zeros(MAX_SIZE)
        self.dr_x_info, self.dr_y_info, self.dr_z_info = tmp.copy(), tmp.copy(), tmp.copy()
        self.t = np.linspace(-MAX_SIZE, 0, num=MAX_SIZE, endpoint=False)

        self.OBJ_PLOT = obj_plot
        self.DR_PLOT = drone_plot

        num_cols = int(self.OBJ_PLOT) + int(self.DR_PLOT)
        if self.OBJ_PLOT:
            self.obj_x_plot = plt.subplot(3, num_cols, 1, title='obj x')
            self.obj_y_plot = plt.subplot(3, num_cols, 2, title='obj y')
            self.obj_z_plot = plt.subplot(3, num_cols, 3, title='obj z')
            self.update_obj_graph()
        if self.DR_PLOT:
            self.drone_x_plot = plt.subplot(3, num_cols, 1 + 3 * int(self.OBJ_PLOT), title='drone x')
            self.drone_y_plot = plt.subplot(3, num_cols, 2 + 3 * int(self.OBJ_PLOT), title='drone y')
            self.drone_z_plot = plt.subplot(3, num_cols, 3 + 3 * int(self.OBJ_PLOT), title='drone z')
            self.update_drone_graph()

    def update_drone(self, x, y, z):
        self.dr_x_info[:-1] = self.dr_x_info[1:]
        self.dr_x_info[-1] = x
        self.dr_y_info[:-1] = self.dr_y_info[1:]
        self.dr_y_info[-1] = y
        self.dr_z_info[:-1] = self.dr_z_info[1:]
        self.dr_z_info[-1] = z
        if self.DR_PLOT:
            self.update_drone_graph()

    def update_obj(self, x, y, z):
        self.obj_x_info[:-1] = self.obj_x_info[1:]
        self.obj_x_info[-1] = x
        self.obj_y_info[:-1] = self.obj_y_info[1:]
        self.obj_y_info[-1] = y
        self.obj_z_info[:-1] = self.obj_z_info[1:]
        self.obj_z_info[-1] = z
        if self.OBJ_PLOT:
            self.update_obj_graph()

    def update_drone_graph(self):
        self.drone_x_plot(self.t, self.dr_x_info)
        self.drone_y_plot(self.t, self.dr_y_info)
        self.drone_z_plot(self.t, self.dr_z_info)
        plt.draw()
        self.drone_x_plot.clear()
        self.drone_y_plot.clear()
        self.drone_z_plot.clear()

    def update_obj_graph(self):
        xt, yt, zt = self.obj_x_info.T, self.obj_y_info.T, self.obj_z_info.T
        for i, mode in enumerate(['P', 'I', 'D']):
            self.obj_x_plot.plot(self.t, xt[i], label=mode)
            self.obj_y_plot.plot(self.t, yt[i], label=mode)
            self.obj_z_plot.plot(self.t, zt[i], label=mode)
        self.obj_x_plot.legend()
        self.obj_y_plot.legend()
        self.obj_z_plot.legend()
        plt.draw()
        self.obj_x_plot.clear()
        self.obj_y_plot.clear()
        self.obj_z_plot.clear()

