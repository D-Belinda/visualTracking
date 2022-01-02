# import the necessary packages
from collections import deque
from imutils.video import VideoStream
from djitellopy import Tello
import pygame
import numpy as np
import argparse
import cv2
import imutils
import time

from hsv_class import hsv_setter
from object_tracking_class import object_tracker
from FrontEnd import FrontEnd

tello = Tello()
tello.connect(False)

tello.streamoff()
tello.streamon()

track = object_tracker()
frame = tello.get_frame_read().frame
track.setFrame(frame)
# time.sleep(2.0)

offset = None

while True:
    # grab the current frame
    frame = tello.get_frame_read().frame
    frame = track.processAll(frame, track.hsv_value)

    # Showing the frame is not included in the class!
    # show the frame to our screen and increment the frame counter
    cv2.imshow("Frame", frame)

    # print the offset
    offset = track.getOffset()
    print(offset)

    #run motion class here to move appropriately
    key = cv2.waitKey(1) & 0xFF
    # counter += 1

    # if the 'q' key is pressed, stop the loop
    if key == ord("q"):
        tello.streamoff()
        tello.end()
        break
