from collections import deque
import numpy as np
import argparse
import cv2
import imutils

class object_tracker:
    def __init__(self):
        """ Maintains the video stream from the drone
            Applies the object tracking using pre-saved HSV values
            Returns the offset and radius of the tracked object
        """
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

        # returning values for tracking offset
        self.xoff = self.yoff = 0
        self.radius = 0

    def processAll(self, frame, hsv_value, framecenter):
        """ Layering a mask and displaying tracked object
            Returns the offset and radius of the tracked object
        """

        # importing the pre-saved HSV values
        self.hsv_value = np.asarray(hsv_value)

        # frame = frame[1] if self.args.get("video", False) else frame
        # if we are viewing a video and we did not grab a frame,
        # then we have reached the end of the video
        if frame is None:
            pass

        # resize the frame, blur it, and convert it to the HSV
        # color space
        blurred = cv2.GaussianBlur(frame, (11, 11), 0)
        hsv_temp = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        # construct a mask for the specified hsv values, then perform
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
            M = cv2.moments(c)
            self.center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
            self.radius = radius

            # only proceed if the radius meets a minimum size
            if radius > 10:
                frame = cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 255), 2)
                frame = cv2.circle(frame, self.center, 5, (0, 0, 255), -1)
                self.pts.appendleft(self.center)

            # FIXME: can the size of the radius also be accumulated
            # FIXME: in self.pts to return dR?

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

                # FIXME: bottom is no longer used, can delete
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

            # get the horizontal and vertical offset
            self.xoff = (self.pts[i][0] - framecenter[0])
            self.yoff = (self.pts[i][1] - framecenter[1])

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

    def getOffset(self):
        """ Returning the values necessary for motion control
            [0]: self.xoff
            [1]: self.yoff
            [2]: self.radius
        """
        return self.xoff, self.yoff, self.radius
