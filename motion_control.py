import cv2
import djitellopy
from collections import deque

class motion_controller:
    def __init__(self):
        self.q = deque() # a queue of circles
    def add(self,cirlce):
        if len(self.q) <= 20:
            self.q.append(cirlce)
