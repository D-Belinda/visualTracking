import time, cv2
from threading import Thread
from djitellopy import Tello
import time

tello = Tello()

tello.connect(False)

keepRecording = True
tello.streamon()
frame_read = tello.get_frame_read()

def videoRecorder():
    # create a VideoWrite object, recording to ./video.avi
    while True:
        # In reality you want to display frames in a separate thread. Otherwise
        #  they will freeze while the drone moves.
        img = frame_read.frame
        img = cv2.resize(img, (480,360))
        cv2.imshow("drone", img)

        # Movement, not used right now
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
            tello.move_forward(30)
        elif key == ord('s'):
            tello.move_back(30)
        elif key == ord('a'):
            tello.move_left(30)
        elif key == ord('d'):
            tello.move_right(30)
        # Rotations
        elif key == ord('l'):
            tello.rotate_clockwise(30)
        elif key == ord('j'):
            tello.rotate_counter_clockwise(30)
        # Elevation
        elif key == ord('i'):
            tello.move_up(30)
        elif key == ord('k'):
            tello.move_down(30)


# we need to run the recorder in a separate thread, otherwise blocking options
#  would prevent frames from getting added to the video
recorder = Thread(target=videoRecorder)
recorder.start()

#tello.takeoff()
#tello.move_up(100)
#tello.rotate_counter_clockwise(360)
#tello.land()
#time.sleep(30);


keepRecording = False
recorder.join()

cv2.destroyAllWindows()
