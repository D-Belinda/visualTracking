import cv2
from imutils.video import VideoStream
from VideoTracking import VideoTracking
import time

vs = VideoStream(0)

tracker = VideoTracking(vs)
tracker.track_frame()



