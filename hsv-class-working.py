# import necessary things
#from collections import deque
from hsv_class import hsv_setter
import cv2
from djitellopy import Tello
import time

tello = Tello()
tello.connect(False)

tello.streamoff()
tello.streamon()

obj = hsv_setter()
time.sleep(2.0)

while True:
    frame = tello.get_frame_read().frame
    #ret, frame = cap.read()
    #if not ret:
    #    break
    obj.get_hsv()

    frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
    obj.display_preview(frame)
