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
from motion-tracking-
# Speed of the drone
S = 30
# Frames per second of the pygame window display
# A low number also results in input lag, as input information is processed once per frame.
FPS = 120

