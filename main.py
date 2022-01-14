from djitellopy import Tello
import cv2
import time
from object_tracking_class import object_tracker
from motion_tracking_class import motionTracking

# Next step ideas
# - change or increase/decrease the speed based on how large the offset is
# - use dX, dY, dR to optimize the speed of the drone
# - how to ensure RC commands are sent/executed more quickly
# - real time HSV adjustment and application (deep learning)

# initialize the Tello object
tello = Tello()
tello.connect(False) # must be false - receiving state packet errors

#a = tello.get_battery()
#print(a)

# in case the stream is already on, turn off and back on
tello.streamoff()
tello.streamon()

# initialize the object tracking class and find the frame center
track = object_tracker()
frame = tello.get_frame_read().frame
framecenter = (frame.shape[1] / 2, frame.shape[0] / 2)
print(framecenter)

# initialize the motion tracking class
move = motionTracking()
offset = None

# start flying
tello.takeoff()

# sleep to allow everything time to get running
time.sleep(1.5)


while True:
    # grab the current frame
    frame = tello.get_frame_read().frame
    frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

    # The frame was also resized in the tello.py BackgroundFrameRead source code
    # Need to ensure center is matching, format: (width, height)
    frame = cv2.resize(frame, (640, 480))
    frame = track.processAll(frame, track.hsv_value, framecenter)

    # show the frame to our screen
    cv2.imshow("Frame", frame)

    # print the offset
    offset = track.getOffset()
    print(offset)

    # moving based on the offset
    move.move(offset)
    move.update(tello)

    key = cv2.waitKey(1) & 0xFF
    # if the 'q' key is pressed, stop the loop and end the program
    if key == ord("q"):
        tello.land()
        tello.streamoff()
        tello.end()
        break

