import numpy as np

# Speed of the drone
S = 10
# Frames per second of the pygame window display
# A low number also results in input lag, as input information is processed once per frame.
FPS = 120


class motionTracking():
    """ Based on offset values returned by object_tracker,
        the drone moves as appropriate
    """
    def __init__(self):
        # Drone velocities between -100~100
        self.for_back_velocity = 0
        self.left_right_velocity = 0
        self.up_down_velocity = 0
        self.yaw_velocity = 0
        self.speed = 10

        self.send_rc_control = False

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

        # only move if there is a significant offset or
        # the object is very close to the drone
        if np.abs(offset[0]) >= 40 or np.abs(offset[1]) >= 30 \
                or offset[2] > 100:
            self.send_rc_control = True
        else:
            return

        # if statement to control the process of moving
        # moving up/down based on y-offset
        if offset[1] < -30:
            self.up_down_velocity = S
        elif offset[1] > 30:
            self.up_down_velocity = -S

        # rotating left/right based on x-offset
        if offset[0] < -40:
            self.yaw_velocity = -S
        elif offset[0] > 40:
            self.yaw_velocity = S

        # moving forward/back based on the radius
        if offset[2] > 100:
            self.for_back_velocity = -S
        elif offset[2] < 50:
            self.for_back_velocity = S

    def update(self, tello):
        """ Update routine. Send velocities to Tello."""
        if self.send_rc_control:
            tello.send_rc_control(self.left_right_velocity, self.for_back_velocity,
                                  self.up_down_velocity, self.yaw_velocity)