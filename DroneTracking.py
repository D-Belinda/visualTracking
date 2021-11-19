# Import things here
from djitellopy import Tello

class DroneTracking:
    # FIXME: How to get center of the video frame?

    def __init__(self, center, radius):
        self.center = center
        self.radius = radius

    def Movement(self):
        # If center(X) < center of video frame,
            #Drone should move left
        # etc...