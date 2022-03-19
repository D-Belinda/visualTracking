import numpy as np

S = 10  # Speed of the drone
T = 1.5  # Turn coefficient - needs to turn faster
E = 1.5  # Elevation coefficient - moving up/down faster
D = 1  # Distance coefficient - moving forward/back faster


# Next step ideas
# - change or increase/decrease the speed based on how large the offset is
# - use dX, dY, dR to optimize the speed of the drone
# - how to ensure RC commands are sent/executed more quickly

class motionTrackingBuffer():
    """ Based on offset values returned by object_tracker,
        the drone moves as appropriate
    """

    def __init__(self):
        # Drone velocities between -100~100
        self.for_back_velocity = 0
        self.left_right_velocity = 0
        self.up_down_velocity = 0
        self.yaw_velocity = 0
        self.speed = S

        self.send_rc_control = False
        self.y_buffer_out = self.y_buffer_in = False
        self.x_buffer_out = self.x_buffer_in = False
        self.r_buffer_out = self.r_buffer_in = False

        self.y_bound = self.x_bound = self.r_bound = None

    def move(self, offset):
        # offset is returned by the object tracking class
        # offset[0]: difference between x-coordinate and center
        # offset[1]: difference between y-coordinate and center
        # offset[2]: radius of the object tracked

        self.for_back_velocity = 0
        self.yaw_velocity = 0
        self.left_right_velocity = 0
        self.up_down_velocity = 0
        self.send_rc_control = False

        self.y_buffer_in = self.y_buffer_out = False
        self.x_buffer_in = self.x_buffer_out = False
        self.r_buffer_in = self.r_buffer_out = False

        # only move if there is a significant offset or
        # the object is very close to the drone
        # set parameters as global variable
        if np.abs(offset[0]) >= 40 or np.abs(offset[1]) >= 30 \
                or offset[2] > 55 or offset[2] < 45:
            self.send_rc_control = True
        else:
            return

        # if statement to control the process of moving

        # rotating left/right based on x-offset
        if np.abs(offset[0]) >= 45:
            self.x_buffer_out = True
            self.x_bound = 35
        elif np.abs(offset[0]) <= 35:
            self.x_buffer_in = True
            self.x_bound = 45
        else:
            self.x_bound = 40

        if offset[0] < -self.x_bound:
            self.yaw_velocity = int(-T * S)  # turn counter clockwise
        elif offset[0] > self.x_bound:
            self.yaw_velocity = int(T * S)  # turn clockwise

        # moving up/down based on y-offset
        if np.abs(offset[1]) >= 35:
            self.y_buffer_out = True
            self.y_bound = 25
        elif np.abs(offset[1]) <= 25:
            self.y_buffer_in = True
            self.y_bound = 35
        else:
            self.y_bound = 30

        if offset[1] < -self.y_bound:
            self.up_down_velocity = int(E * S)  # move up
        elif offset[1] > self.y_bound:
            self.up_down_velocity = int(-E * S)  # move down

        # moving forward/back based on the radius
        if np.abs(offset[2]) >= 55:
            self.r_buffer_out = True
            self.r_bound = 45
        elif np.abs(offset[2]) <= 45:
            self.r_buffer_in = True
            self.r_bound = 55
        else:
            self.r_bound = 50

        if offset[2] > self.r_bound:
            self.for_back_velocity = int(-D * S)  # move backwards
        elif offset[2] < self.r_bound:
            self.for_back_velocity = int(D * S)  # move forwards

    def update(self, tello):
        """ Update routine. Send velocities to Tello."""
        if self.send_rc_control:
            tello.send_rc_control(self.left_right_velocity, self.for_back_velocity,
                                  self.up_down_velocity, self.yaw_velocity)

        # else:
        # print(0, 0, 0, 0)
