from djitellopy import Tello
import cv2
import pygame
import pandas as pd
import numpy as np
import os
import time
from object_tracking_class import ObjectTracker
from motion_control import MotionController
from data_logger import Logger

# Speed of the drone
S = 10
# Frames per second of the pygame window display
# A low number also results in input lag, as input information is processed once per frame.
FPS = 30
LOG = False


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
        self.vz = 0
        self.vx = 0
        self.vy = 0
        self.vyaw = 0
        self.v = 0.0, 0.0, 0.0  # yaw, up/down, forward/backward
        self.speed = 10  # do not change this
        self.recording = False

        self.send_rc_control = False

        # create update timer
        pygame.time.set_timer(pygame.USEREVENT + 1, 1000 // FPS)

        self.ot = ObjectTracker()
        self.motion_controller = MotionController(FPS)
        if LOG:
            self.logger = Logger(obj_plot=False, drone_plot=False)

    def process_frame(self, frame):
        # Displaying battery
        battery_text = "Battery: {}%".format(self.tello.get_battery())
        frame = cv2.putText(frame, battery_text, (5, 720 - 5), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        frame = cv2.putText(frame, "velocities: " + str(self.v), (5, 45), cv2.FONT_HERSHEY_SIMPLEX, 1,
                            (255, 255, 255), 2)
        frame = np.rot90(frame)
        frame = np.flipud(frame)

        frame = cv2.resize(frame, (int(frame.shape[1] * 0.9), int(frame.shape[0] * 0.9)))
        frame = pygame.surfarray.make_surface(frame)
        return frame

    def run(self):
        global instruction_counter
        self.tello.connect()
        self.tello.set_speed(self.speed)

        # In case streaming is on. This happens when we quit this program without the escape key.
        self.tello.streamoff()
        self.tello.streamon()
        # self.tello.send_keepalive()
        frame_read = self.tello.get_frame_read()

        should_stop = False

        record = []
        counter = max([int(e.split('.')[0]) for e in os.listdir('jinran/img') if e[0] != '.']+[1])
        start = counter
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
                        self.motion_controller.clear_data()
                        self.send_rc_control = True
                    elif event.key == pygame.K_SPACE:
                        should_stop = True
                    elif event.key == pygame.K_r:
                        self.recording = not self.recording
                        print(self.recording)
                        if self.recording:
                            start = counter
                        if not self.recording:
                            csv_name = f'jinran/{start}-{counter}.csv'
                            pd.DataFrame(record).to_csv(csv_name)

            self.screen.fill([0, 0, 0])

            orig_frame = frame_read.frame
            frame, rect = self.ot.get_rect(orig_frame)
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)  # Commented out to color correct

            self.motion_controller.add_location(rect)
            self.v = self.motion_controller.instruct(diagnostic=False)

            if self.recording:
                img_name = f'jinran/img/{counter}.jpg'
                print(f'{img_name},{cv2.imwrite(img_name, orig_frame)}')
                counter += 1
                record.append((img_name, time.time(), rect, self.v))
                if len(record) > 500:
                    csv_name = f'jinran/{start}-{counter}.csv'
                    pd.DataFrame(record).to_csv(csv_name)
                    record = []
                    start = counter

            if self.tello.is_flying:
                self.v = list(map(int, self.v))
                self.vx, self.vy, self.vz = self.v

            # logging data
            if LOG:
                self.logger.update_drone(np.array([self.vx, self.tello.get_speed_x()]),
                                         np.array([self.vy, self.tello.get_speed_y()]),
                                         np.array([self.vz, self.tello.get_speed_z()]))
                self.logger.update_obj(self.motion_controller.get_x_info(),
                                       self.motion_controller.get_y_info(),
                                       self.motion_controller.get_z_info())

            frame = self.process_frame(frame)
            self.screen.blit(frame, (0, 0))
            pygame.display.update()

            time.sleep(1 / FPS)

        self.tello.land()
        self.send_rc_control = False
        self.tello.end()

    def update(self):
        if not self.send_rc_control:
            return
        self.tello.send_rc_control(self.vx, self.vz,
                                   self.vy, self.vyaw)


def main():
    frontend = FrontEnd()

    # run frontend
    frontend.run()


if __name__ == '__main__':
    main()
