# import the necessary packages
from collections import deque
from imutils.video import VideoStream
import numpy as np
import argparse
import cv2
import imutils
import time

from hsv_class import hsv_setter
from object_tracking_class import object_tracker

track = object_tracker()
vs = VideoStream(0).start()

while True:
    # grab the current frame
    frame = vs.read()
    frame = track.processAll(frame, track.hsv_value)

    # Showing the frame is not included in the class!
    # show the frame to our screen and increment the frame counter
    cv2.imshow("Frame", frame)

    key = cv2.waitKey(1) & 0xFF
    # counter += 1

    # if the 'q' key is pressed, stop the loop
    if key == ord("q"):
        vs.stop()
        vs.release()
        break
