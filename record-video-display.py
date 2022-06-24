import time, cv2
from threading import Thread
import numpy as np
from djitellopy import Tello
from object_tracking_class import object_tracker

DISTANCE = 10   # Distance (cm) the drone moves per command

tello = Tello()

tello.connect(False)

keepRecording = True
tello.streamoff()   # Call just in case
tello.streamon()

track = object_tracker()
frame = tello.get_frame_read().frame
framecenter = (frame.shape[1] / 2, frame.shape[0] / 2)
print(framecenter)

offset_coords = []  # Empty array to store all sets of coordinates

def videoRecorder():

    while True:
        # grab the current frame
        frame = tello.get_frame_read().frame
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

        # The frame was also resized in the tello.py BackgroundFrameRead source code
        # Need to ensure center is matching, format: (width, height)
        frame = cv2.resize(frame, (640, 480))
        frame = track.processAll(frame, track.hsv_value, framecenter)

        # show the frame to the screen
        cv2.imshow("Frame", frame)

        # Discrete movement
        key = cv2.waitKey(1) & 0xff
        if key == 27:  # ESC
            break
        # Takeoff and landing
        elif key == ord('t'):
            tello.takeoff()
        elif key == ord('g'):
            tello.land()
        # Directions
        elif key == ord('w'):
            tello.move_forward(DISTANCE)
        elif key == ord('s'):
            tello.move_back(DISTANCE)
        elif key == ord('a'):
            tello.move_left(DISTANCE)
        elif key == ord('d'):
            tello.move_right(DISTANCE)
        # Rotations
        elif key == ord('l'):
            tello.rotate_clockwise(DISTANCE)
        elif key == ord('j'):
            tello.rotate_counter_clockwise(DISTANCE)
        # Elevation
        elif key == ord('i'):
            tello.move_up(DISTANCE)
        elif key == ord('k'):
            tello.move_down(DISTANCE)
        # Recording data
        elif key == ord('r'):
            offset = track.getOffset()
            print(offset)

            offset_coords.append(offset)

# we need to run the recorder in a separate thread, otherwise blocking options
#  would prevent frames from getting added to the video
recorder = Thread(target=videoRecorder)
recorder.start()

keepRecording = False
recorder.join()

# save data about object's coordinates and size into 'coordinates.csv'
# this file will later be imported directly into MATLAB and graphed
with open('coordinates.csv', 'w') as csvfile:
    np.savetxt(csvfile, offset_coords)
    print('Coordinates saved')

cv2.destroyAllWindows()
