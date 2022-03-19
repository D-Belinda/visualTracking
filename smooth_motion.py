from djitellopy import Tello
import cv2
import pygame
import numpy as np
import time
from object_tracking_class import object_tracker
from hsv_class import hsv_setter
from motion_control import motion_controller

# Speed of the drone
S = 10
# Frames per second of the pygame window display
# A low number also results in input lag, as input information is processed once per frame.
FPS = 80


class FrontEnd(object):
    """ Maintains the Tello display and moves it through the keyboard keys.
        Press escape key to quit.
        The controls are:
            - T: Takeoff
            - L: Land
            - Arrow keys: Forward, backward, left and right.
            - A and D: Counter clockwise and clockwise rotations (yaw)
            - W and S: Up and down.
            - M: enable hand control
            - C: enable camera control (developing)
    """
    handControl = True

    def __init__(self):
        # Init pygame
        pygame.init()

        # Create pygame window
        pygame.display.set_caption("Tello video stream")
        self.screen = pygame.display.set_mode([int(960*0.9), int(720*0.9)])

        # Init Tello object that interacts with the Tello drone
        self.tello = Tello()

        # Drone velocities between -100~100
        self.for_back_velocity = 0
        self.left_right_velocity = 0
        self.up_down_velocity = 0
        self.yaw_velocity = 0
        self.acceleration = 0.0, 0.0, 0.0
        self.speed = 10 # do not change this

        self.send_rc_control = False

        # create update timer
        pygame.time.set_timer(pygame.USEREVENT + 1, 1000 // FPS)

        self.ot = object_tracker()
        self.hsv_control = hsv_setter()
        self.motion_controller = motion_controller(FPS)

    def run(self):

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

            self.screen.fill([0, 0, 0])

            frame = frame_read.frame
            # frame = cv2.resize(frame, (int(frame.shape[1]*0.9), int(frame.shape[0]*0.9)), interpolation=cv2.INTER_AREA)

            self.hsv_control.display_preview(frame)
            hsv_values = self.hsv_control.get_hsv(frame)
            frame = self.ot.process_all(frame, hsv_values)

            # Displaying battery
            batteryText = "Battery: {}%".format(self.tello.get_battery())
            frame = cv2.putText(frame, batteryText, (5, 720 - 5), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
            '''
            # Displaying speed and acceleration
            text_x = "X Spd: {} cm/s".format(self.tello.get_speed_x()) + "  Acc: {} cm/s^2".format(self.tello.get_acceleration_x())
            frame = cv2.putText(frame, text_x, (5, 720 - 95), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
            text_y = "Y Spd: {} cm/s".format(self.tello.get_speed_y()) + "  Acc: {} cm/s^2".format(self.tello.get_acceleration_y())
            frame = cv2.putText(frame, text_y, (5, 720 - 65), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
            text_z = "Z Spd: {} cm/s".format(self.tello.get_speed_z()) + "  Acc: {} cm/s^2".format(self.tello.get_acceleration_z())
            frame = cv2.putText(frame, text_z, (5, 720 - 35), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
            '''
            # display hand control state
            hand_control_text = "Hand control: " + ("Enabled" if self.handControl else "Disabled")
            frame = cv2.putText(frame, hand_control_text, (5, 35), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
            flying_text = "In Flight: " + ("True" if self.tello.is_flying else "False")
            frame = cv2.putText(frame, flying_text, (5, 65), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)


            # display location of the target
            if len(self.ot.get_circle()) != 0:
                frame = cv2.putText(frame, "Tx={}  Ty={}".format(int(self.ot.get_circle()[0]), int(self.ot.get_circle()[1])), (5, 720 - 35), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)


            self.motion_controller.add_location(self.ot.get_circle())

            for event in pygame.event.get():
                if event.type == pygame.USEREVENT + 1:
                    self.update()
                elif event.type == pygame.QUIT:
                    should_stop = True
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:
                        should_stop = True
                    elif event.key == pygame.K_m:
                        self.handControl = True
                    elif event.key == pygame.K_c:
                        self.handControl = False
                    elif self.handControl:
                        self.keydown(event.key)
                elif event.type == pygame.KEYUP and (self.handControl or event.key == pygame.K_l):
                    self.keyup(event.key)

            self.acceleration = self.motion_controller.instruct()
            # display acceleration (x, y, forward/backward)
            frame = cv2.putText(frame, "accelerations: " + str(self.acceleration), (5, 95), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
            self.yaw_velocity += int(self.acceleration[0] / FPS)

            #frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            frame = np.rot90(frame)
            frame = np.flipud(frame)

            frame = cv2.resize(frame, (int(frame.shape[1] * 0.9), int(frame.shape[0] * 0.9)))
            frame = pygame.surfarray.make_surface(frame)
            self.screen.blit(frame, (0, 0))
            pygame.display.update()

            time.sleep(1 / FPS)

        # Call it always before finishing. To deallocate resources.
        self.tello.end()

    def keydown(self, key):
        """ Update velocities based on key pressed
        Arguments:
            key: pygame key
        """
        if key == pygame.K_UP:  # set forward velocity
            self.for_back_velocity = S
        elif key == pygame.K_DOWN:  # set backward velocity
            self.for_back_velocity = -S
        elif key == pygame.K_LEFT:  # set left velocity
            self.left_right_velocity = -S
        elif key == pygame.K_RIGHT:  # set right velocity
            self.left_right_velocity = S
        elif key == pygame.K_w:  # set up velocity
            self.up_down_velocity = S
        elif key == pygame.K_s:  # set down velocity
            self.up_down_velocity = -S
        elif key == pygame.K_a:  # set yaw counter clockwise velocity
            self.yaw_velocity = -S
        elif key == pygame.K_d:  # set yaw clockwise velocity
            self.yaw_velocity = S

    def keyup(self, key):
        """ Update velocities based on key released
        Arguments:
            key: pygame key
        """
        if key == pygame.K_UP or key == pygame.K_DOWN:  # set zero forward/backward velocity
            self.for_back_velocity = 0
        elif key == pygame.K_LEFT or key == pygame.K_RIGHT:  # set zero left/right velocity
            self.left_right_velocity = 0
        elif key == pygame.K_w or key == pygame.K_s:  # set zero up/down velocity
            self.up_down_velocity = 0
        elif key == pygame.K_a or key == pygame.K_d:  # set zero yaw velocity
            self.yaw_velocity = 0
        elif key == pygame.K_t:  # takeoff
            self.tello.takeoff()
            self.send_rc_control = True
        elif key == pygame.K_l:  # land
            self.tello.land()
            self.send_rc_control = False

    def update(self):
        """ Update routine. Send velocities to Tello."""
        if self.send_rc_control:
            self.tello.send_rc_control(self.left_right_velocity, self.for_back_velocity,
                                       self.up_down_velocity, self.yaw_velocity)


def main():
    print("initiating...")
    frontend = FrontEnd()

    # run frontend
    frontend.run()


if __name__ == '__main__':
    main()
