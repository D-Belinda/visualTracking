from djitellopy import Tello
import cv2
import pygame
import numpy as np
import time
import os
from object_tracking_class import ObjectTracker

"""
To use this file:
Do not need to takeoff the drone. Keep the card in the drone's field of view, then press 'R' to make it take a picture 
every 5 frames. To stop recording, press 'r' again
"""

# Speed of the drone
S = 10
# Frames per second of the pygame window display
# A low number also results in input lag, as input information is processed once per frame.
FPS = 40
INTVERVAL = 15  # take a pic every 5 frames
interval_counter = 0


class FrontEnd(object):
    """ Maintains the Tello display and moves it through the keyboard keys.
        Press escape key to quit.
        The controls are:
            - RETURN: Takeoff
            - SPACE: Land
    """
    handControl = True

    def __init__(self):
        # Init pygame
        pygame.init()

        # Create pygame window
        pygame.display.set_caption("Tello video stream")
        self.screen = pygame.display.set_mode([int(960 * 0.9), int(720 * 0.9)])

        # Init Tello object that interacts with the Tello drone
        self.tello = Tello()

        # Drone velocities between -100~100
        self.for_back_velocity = 0
        self.left_right_velocity = 0
        self.up_down_velocity = 0
        self.yaw_velocity = 0
        self.v = 0.0, 0.0, 0.0  # yaw, up/down, forward/backward
        self.speed = 10  # do not change this

        self.send_rc_control = False
        self.recording = False

        # create update timer
        pygame.time.set_timer(pygame.USEREVENT + 1, 1000 // FPS)

        # Init object tracker
        self.ot = ObjectTracker()

    def process_frame(self, frame):
        # Displaying battery
        battery_text = "Battery: {}%".format(self.tello.get_battery())
        frame = cv2.putText(frame, battery_text, (5, 720 - 5), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        frame = cv2.putText(frame, "velocities: " + str(self.v), (5, 45), cv2.FONT_HERSHEY_SIMPLEX, 1,
                            (255, 255, 255), 2)
        frame = cv2.putText(frame, 'rect: ' + str(self.rect), (5, 75), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        frame = np.rot90(frame)
        frame = np.flipud(frame)

        frame = cv2.resize(frame, (int(frame.shape[1] * 0.9), int(frame.shape[0] * 0.9)))
        frame = pygame.surfarray.make_surface(frame)
        return frame

    def run(self):
        global INTVERVAL, interval_counter
        img_counter = max([int(e.split('.')[0]) for e in os.listdir('left_right_dataset') if 'jpg' in e]+[0])+1
        print(img_counter)

        self.tello.connect()
        self.tello.set_speed(self.speed)

        # In case streaming is on. This happens when we quit this program without the escape key.
        self.tello.streamoff()
        self.tello.streamon()
        frame_read = self.tello.get_frame_read()

        should_stop = False
        while not should_stop:
            if frame_read.stopped:
                break

            for event in pygame.event.get():
                if event.type == pygame.USEREVENT + 1:
                    self.update()
                elif event.type == pygame.QUIT:
                    break
                elif event.type == pygame.KEYUP:
                    if event.key == pygame.K_RETURN:
                        self.tello.takeoff()
                        self.send_rc_control = True
                    elif event.key == pygame.K_SPACE:
                        should_stop = True
                    elif event.key == pygame.K_r:
                        self.recording = not self.recording

            self.screen.fill([0, 0, 0])

            frame = frame_read.frame
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            frame, self.rect = self.ot.get_rect(frame)
            # frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

            interval_counter += 1
            if INTVERVAL == interval_counter:
                if self.recording:
                    print(f'img{img_counter} status:{cv2.imwrite(f"left_right_dataset/{str(img_counter)}.jpg", frame)}')
                img_counter += 1
                interval_counter = 0

            gen_text = f"Generating: {self.recording}  {self.tello.get_battery()}"
            frame = cv2.putText(frame, gen_text, (5, 720 - 35), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)

            frame = self.process_frame(frame)
            self.screen.blit(frame, (0, 0))
            pygame.display.update()

            time.sleep(1 / FPS)

        # Call it always before finishing. To deallocate resources.
        self.tello.land()
        self.send_rc_control = False
        self.tello.end()

    def update(self):
        if not self.send_rc_control:
            return
        self.tello.send_rc_control(self.left_right_velocity, self.for_back_velocity,
                                   self.up_down_velocity, self.yaw_velocity)


def main():
    frontend = FrontEnd()

    # run frontend
    frontend.run()


if __name__ == '__main__':
    main()
