#import the necessary packages
from djitellopy import Tello
from collections import deque
import argparse
import imutils
import cv2
import pygame
import numpy as np
import time

from object_tracking_class import object_tracker
from FrontEnd import FrontEnd

# Speed of the drone
S = 5
# Frames per second of the pygame window display
# A low number also results in input lag, as input information is processed once per frame.
FPS = 120

class motionTracking():

    def __init__(self):
        # Init pygame
        #pygame.init()

        # Drone velocities between -100~100
        self.for_back_velocity = 0
        self.left_right_velocity = 0
        self.up_down_velocity = 0
        self.yaw_velocity = 0
        self.speed = 10

        self.send_rc_control = True

    def move(self, offset):
        self.for_back_velocity = 0
        self.yaw_velocity = 0
        self.left_right_velocity = 0
        self.up_down_velocity = 0

        # if statement to control the process
        if offset[0] == 0 and offset[1] == 0:
            self.left_right_velocity = 0
            self.up_down_velocity = 0

        elif offset[0] < 0:
            self.left_right_velocity = -S

            if offset[1] < 0:
                self.up_down_velocity = S
            else:
                self.up_down_velocity = -S

        elif offset[0] > 0:
            self.left_right_velocity = S

            if offset[1] < 0:
                self.up_down_velocity = S
            else:
                self.up_down_velocity = -S

        #elif offset[1] < 0:
            #self.up_down_velocity = S
        #elif offset[1] > 0:
            #self.up_down_velocity = -S

        #elif offset[0] < 0:
            #self.left_right_velocity = -S
        #elif offset[0] > 0:
            #self.left_right_velocity = S

    def update(self, tello):
        """ Update routine. Send velocities to Tello."""
        #tello.set_speed(self.speed)

        if self.send_rc_control:
            tello.send_rc_control(self.left_right_velocity, self.for_back_velocity,
                self.up_down_velocity, self.yaw_velocity)