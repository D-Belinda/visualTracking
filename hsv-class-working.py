from hsv_class import hsv_setter
import cv2
from djitellopy import Tello
import time

# initialize the tello
tello = Tello()
tello.connect(False) # must set to False, will cause state packet error

# turn stream off and then on
tello.streamoff()
tello.streamon()

# initialize HSV class
obj = hsv_setter()
time.sleep(2.0)

while True:
    frame = tello.get_frame_read().frame
    obj.get_hsv()

    # show frame
    frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
    obj.display_preview(frame)
