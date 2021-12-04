# import necessary things
#from collections import deque
from hsv_class import hsv_setter
import cv2
obj = hsv_setter()
cap = cv2.VideoCapture(0)
cap.set(3,1280)
cap.set(4,720)

while True:
    ret, frame = cap.read()
    if not ret:
        break
    obj.get_hsv()
    obj.display_preview(frame)
