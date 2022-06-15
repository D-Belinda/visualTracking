# USAGE
# python object_tracking.py --video object_tracking_example.mp4
# python object_tracking.py

# import the necessary packages
from collections import deque
import numpy as np
import argparse
import cv2
import imutils
import time


class ObjectTracker:
    def __init__(self):
        # construct the argument parse and parse the arguments
        self.ap = argparse.ArgumentParser()
        self.ap.add_argument("-v", "--video",
                             help="path to the (optional) video file")
        self.ap.add_argument("-b", "--buffer", type=int, default=32,
                             help="max buffer size")
        self.args = vars(self.ap.parse_args())
        # initialize the list of tracked points, the frame counter,
        # and the coordinate deltas
        self.pts = deque(maxlen=self.args["buffer"])
        self.center = None
        self.counter = 0
        (self.dX, self.dY) = (0, 0)
        self.direction = ""
        self.hsv_value = np.load('hsv_value.npy')
        self.circle = ()
        time.sleep(2.0)

    def process_all(self, frame, hsv_value):
        self.hsv_value = np.asarray(hsv_value)

        frame = frame[1] if self.args.get("video", False) else frame
        # if we are viewing a video and we did not grab a frame,
        # then we have reached the end of the video
        if frame is None:
            pass
        # resize the frame, blur it, and convert it to the HSV
        # color space
        # frame = imutils.resize(frame, width=600)
        blurred = cv2.GaussianBlur(frame, (11, 11), 0)
        hsv_temp = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        # construct a mask for the color "green", then perform
        # a series of dilations and erosions to remove any small
        # blobs left in the mask
        mask = cv2.inRange(hsv_temp, self.hsv_value[0], self.hsv_value[1])
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)

        # find contours in the mask and initialize the current
        # (x, y) center of the ball
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                                cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)

        # only proceed if at least one contour was found
        if len(cnts) > 0:
            # find the largest contour in the mask, then use
            # it to compute the minimum enclosing circle and
            # centroid
            c = max(cnts, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            self.circle = (x, y, radius)
            M = cv2.moments(c)
            self.center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
            # only proceed if the radius meets a minimum size
            if radius > 10:
                frame = cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 255), 2)
                frame = cv2.circle(frame, self.center, 5, (0, 0, 255), -1)
                self.pts.appendleft(self.center)

        # loop over the set of tracked points
        for i in np.arange(1, len(self.pts)):
            if self.pts[i - 1] is None or self.pts[i] is None:
                continue
            # check to see if enough points have been accumulated in
            # the buffer
            if self.counter >= 10 and i == 1 and self.pts[-10] is not None:
                # compute the difference between the x and y
                # coordinates and re-initialize the direction
                # text variables
                self.dX = self.pts[-10][0] - self.pts[i][0]
                self.dY = self.pts[-10][1] - self.pts[i][1]
                (dirX, dirY) = ("", "")
                if np.abs(self.dX) > 20:
                    dirX = "East" if np.sign(self.dX) == 1 else "West"

                # ensure there is significant movement in the
                # y-direction
                if np.abs(self.dY) > 20:
                    dirY = "North" if np.sign(self.dY) == 1 else "South"

                # handle when both directions are non-empty
                if dirX != "" and dirY != "":
                    self.direction = "{}-{}".format(dirY, dirX)

                # otherwise, only one direction is non-empty
                else:
                    self.direction = dirX if dirX != "" else dirY

            # otherwise, compute the thickness of the line and
            # draw the connecting lines
            cv2.putText(frame, self.direction, (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
                        0.65, (0, 0, 255), 3)
            cv2.putText(frame, "dx: {}, dy: {}".format(self.dX, self.dY),
                        (10, frame.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX,
                        0.35, (0, 0, 255), 1)
            thickness = int(np.sqrt(self.args["buffer"] / float(i + 1)) * 2.5)
            frame = cv2.line(frame, self.pts[i - 1], self.pts[i], (255, 255, 255), thickness)
        return frame

    def get_circle(self):
        return self.circle
