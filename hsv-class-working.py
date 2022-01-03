# import necessary things
#from collections import deque
from hsv_class import hsv_setter
import cv2
from djitellopy import Tello

tello = Tello()
tello.connect(False)

tello.streamoff()
tello.streamon()

#frame = tello.get_frame_read().frame

obj = hsv_setter()

#cap = cv2.VideoCapture(0)
#cap.set(3,1280)
#cap.set(4,720)

while True:
    frame = tello.get_frame_read().frame
    #ret, frame = cap.read()
    #if not ret:
    #    break
    obj.get_hsv()

    frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
    obj.display_preview(frame)
