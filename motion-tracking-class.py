#import the necessary packages
from djitellopy import Tello
import cv2
import pygame
import numpy as np
import time

from object_tracking_class import object_tracker

class AutoTracking(object_tracker):
