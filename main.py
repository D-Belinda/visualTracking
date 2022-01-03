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
from motion_tracking_class import motionTracking

tello = Tello()
tello.connect(False)

tello.streamoff()
tello.streamon()

tello.takeoff()
tello.set_speed(5)

track = object_tracker()
frame = tello.get_frame_read().frame
framecenter = (frame.shape[1] / 2, frame.shape[0] / 2)
print(framecenter)

move = motionTracking()
offset = None

while True:
    # grab the current frame
    frame = tello.get_frame_read().frame
    frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

    # Frame was also resized in the tello.py BackgroundFrameRead source code
    # Need to ensure center is matching, (width, height)
    frame = cv2.resize(frame, (640, 480))
    frame = track.processAll(frame, track.hsv_value, framecenter)

    # Showing the frame is not included in the class!
    # show the frame to our screen and increment the frame counter
    cv2.imshow("Frame", frame)

    # print the offset
    offset = track.getOffset()
    print(offset)
    time.sleep(1 / 60)

    #run motion class here to move appropriately
    move.move(offset)
    move.update(tello)

    key = cv2.waitKey(1) & 0xFF
    # counter += 1

    # if the 'q' key is pressed, stop the loop
    if key == ord("q"):
        tello.land()
        tello.streamoff()
        tello.end()
        break

