from djitellopy import Tello
import cv2
import time
from object_tracking_class import object_tracker
from motion_tracking_class_buffer import motionTrackingBuffer

FPS = 20

# Next step ideas
# - change or increase/decrease the speed based on how large the offset is
# - use dX, dY, dR to optimize the speed of the drone
#       consider past inputs, ignore large jumps between frames
# - how to ensure RC commands are sent/executed more quickly
# - real time HSV adjustment and application (deep learning)

# initialize the Tello object
tello = Tello()
tello.connect(False) # must be false - receiving state packet errors

# in case the stream is already on, turn off and back on again
tello.streamoff()
tello.streamon()

# initialize the object tracking class and find the frame center
# Note: I changed the default frame settings for the Tello so it would match
#       the cv2.imshow defaults >> (640, 480)
track = object_tracker()
cap = cv2.VideoCapture('object-control.MOV')
frame = cap.read()
framecenter = (frame.shape[1] / 2, frame.shape[0] / 2)
print(framecenter)

# initialize the motion tracking class
move = motionTrackingBuffer()
offset = None

# start flying
tello.takeoff()

# sleep to allow everything time to get running
time.sleep(1.5)


while True:
    # grab the current frame
    frame = cap.read()
    frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

    # The frame was also resized in the tello.py BackgroundFrameRead source code
    # Need to ensure center is matching, format: (width, height)
    frame = cv2.resize(frame, (640, 480))
    frame = track.processAll(frame, track.hsv_value, framecenter)

    # show the frame to the screen
    cv2.imshow("Frame", frame)

    # print the offset
    offset = track.getOffset()
    print(offset)

    # moving based on the offset
    move.move(offset)
    move.update(tello)

    time.sleep(1/FPS)

    key = cv2.waitKey(1) & 0xFF
    # if the 'q' key is pressed, stop the loop and end the program
    if key == ord("q"):
        tello.land()
        tello.streamoff()
        tello.end()
        break

# send command to the drone --> check response time >> what is the delay?
# control at a fast rate >> see what is causing the delay
# buffer zone >> common control question >> continue past then stop, wait until it goes back

# track one dimension first, fix one variable at  a time
# reduce the frame rate (10 FPS) --> object is moving slowly >> simplified version
# where and what is the delay >> consider constraints

# comment >> move == delay 1 - constant
# motor rotation == delay 2 - based on speed

# my job == control problem
# run too fast, include a freeze/sleep time
