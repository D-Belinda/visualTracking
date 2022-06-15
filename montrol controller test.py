import motion_control
import numpy as np
ms = motion_control.MotionController(60)
v = np.array([0,0,0])
for i in range(0,50,1):
    ms.add_location((i,0,0))
    a = ms.instruct(True)
    #print(a)
    v = np.add(v, np.array(a))
    print("v:" + str(v))
print("here")
for i in range(50,0,-2):
    ms.add_location((i,0,0))
    a = ms.instruct(True)
    #print(a)
    v = np.add(v, np.array(a))
    print("v:" + str(v))
