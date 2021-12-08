import cv2
import djitellopy
from collections import deque

class motion_controller:
    def __init__(self):
        self.q = deque()
