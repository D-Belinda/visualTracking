import numpy as np

S = 20      # Speed of the drone
T = 0.75    # Turn coefficient - needs to turn faster
E = 1       # Elevation coefficient - moving up/down faster

# Next step ideas
# - change or increase/decrease the speed based on how large the offset is
# - use dX, dY, dR to optimize the speed of the drone
# - how to ensure RC commands are sent/executed more quickly

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
        if np.abs(offset[0]) >= 40 or np.abs(offset[1]) >= 20 \
                or offset[2] > 80 or offset[2] < 65:
            self.send_rc_control = True
        else:
            return

        # if statement to control the process of moving

        # rotating left/right based on x-offset
        if offset[0] < -40:
            self.yaw_velocity = int(-T*S)    # turn counter clockwise
        elif offset[0] > 40:
            self.yaw_velocity = int(T*S)   # turn clockwise

        # moving up/down based on y-offset
        #if offset[1] < -20:
            #self.up_down_velocity = int(E*S)   # move up
        #elif offset[1] > 20:
            #self.up_down_velocity = int(-E*S)  # move down

        # moving forward/back based on the radius
        #if offset[2] > 80:
            #self.for_back_velocity = -S     # move backwards
        #elif offset[2] < 65:
            #self.for_back_velocity = S      # move forwards

    def update(self, tello):
        """ Update routine. Send velocities to Tello."""
        if self.send_rc_control:
            tello.send_rc_control(self.left_right_velocity, self.for_back_velocity,
                                  self.up_down_velocity, self.yaw_velocity)
            print(self.left_right_velocity, self.for_back_velocity,
                  self.up_down_velocity, self.yaw_velocity)
        # else:
            # print(0, 0, 0, 0)