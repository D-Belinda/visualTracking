*Use this README as a message board to log any changes or ideas, keep track of progress*

# visualTracking
Autonomously identify an object and track it using the DJI Tello drone
* [RC Car Build Instructions](https://racecarj.com/pages/build-instructions)
* [DJI Tellopy Documentation](https://djitellopy.readthedocs.io/en/latest/tello/)

# Object Detection
* HSV colorspace & opencv
  * Limits tracking to relatively static environments, fails in changing brightness + background interference
  * Objects can only be one color, affected by textures and patterns 

* YOLOv5n
  * Bounding box v. circle affects stability of the control portion
  * Model trained for the green notecard, PID constants tuned
  * Model in training for the RC car
  
* Pose Estimation 
 * Turning 
 * Needs more training and PID tuning

## To-Do
- [x] Retune PID constants to account for YOLO identification
- [x] Perfect identification + tracking with the green notecard 
- [x] Gather dataset for RC car (front/left/right/back/top views)
- [x] Annotate dataset for RC car
- [x] Train model for RC car
- [ ] Retune PID constants for RC car
- [ ] Tune PID constants for pose estimation

# Motion Control
* PID Controller
  * Smoother motion, but still has a noticeable delay
  * PID constants can still be fine-tuned

## To-Do
- [ ] Map distance to pixel relationship
 - [ ] Use pandas
- [ ] Log error graphs to help with PID tuning
- [ ] Test performance for sudden movements and edge cases
- [ ] Measure wireless communication delay, see if it is affection the motion delay
- [ ] Create control diagrams

# Ideas
* Fuzzy logic and fuzzy-PID controllers
* Split-Model method (?), try to lower computational time for GPU/CPU
* 
