import threading
import serial
import time
import argparse
import mediapipe as mp
import cv2
import numpy as np
import random
import math

#from __future__ import print_function

class MovingAverageXY:
    def __init__(self, buff_len):
        self.__buff_len = buff_len
        self.__buff = []
        [self.__buff.append([0,0]) for i in range(0, self.__buff_len)]
        self.__accumulator = [0,0]
        self.__ptr = 0

    def get_val(self):
        return [self.__accumulator[i] / self.__buff_len for i in [0,1]]

    def push(self, x):
        for i in [0,1]:
            self.__accumulator[i] -= self.__buff[self.__ptr][i]
            self.__accumulator[i] += x[i]

        self.__buff[self.__ptr] = x

        self.__ptr = (self.__ptr + 1) % self.__buff_len

        return self.get_val()


class Hexibe:
    def __init__(self):
        self.shutdown = False
        self.worker = threading.Thread(target=self.__worker)

        self.__target_position      = [0,0,50]

        self.__current_position     = [0,0,50]       # [step] This is the real position of motors, in steps
        self.__internal_position    = [0,0,50]         # [step] This is the real-time desired position, aka the numerical integral of speed

        self.__current_speed        = [0,0]         # [step/dt] aka steps per time step [0;1]
        self.__top_speed            = [1/2.5,1/5.2,0.1]     # [step/dt] This is the maximum speed of each motor

        self.__max_acc              = [1/35000,1/20000]         # [step/dt²] This is the maximum acceleration of each motor

        self.__arrived              = False         # True if target=current
        self.__arrived_cv           = threading.Condition() # Notifies arrived change
        self.rgb                    = [0,0,0]
        self.__rgb                  = [0,0,0]


        self.step_per_unit          = [8/1.8*(60/20), 360/40/(1.8/8)]

        self.unit_ranges            = [[-180,180], [-140, +140]]
        self.ranges                 = [[0,0],[0,0]]

        for axis in [0,1]:
            for bound in [0,1]:
                self.ranges[axis][bound] = int(self.unit_ranges[axis][bound] * self.step_per_unit[axis])

        self.positive_dir_cmd       = [[0b10, 0b11],[[0b10, 0b11]]]

    def __brake(self):
        # Calculates if each motor should reduce the speed to not overshoot with constant deceleration braking
        # The duration of braking phase is __current_speed/__max_acc
        # During the braking phase, the steps done are (__current_speed**2)/(2*__max_acc)

        return [abs(self.__target_position[i] - self.__current_position[i]) < (self.__current_speed[i]**2)/(2*self.__max_acc[i]) for i in [0,1]]

    def __mysign(self, x):
        if(x == 0):
            return 1
        return np.sign(x)

    def __cart2pol(self, x, y):
        rho = np.sqrt(x**2 + y**2) * self.__mysign(x)
        phi = np.sign(y)*90 if x == 0 else np.arctan(y/x)/(2*np.pi)*360
        return(rho, phi)

    def open(self, device):
        self.ser = serial.Serial(device, 3000000)
        self.worker.start()

    def __worker(self):
        buff = []
        dirs = [-1, +1, 0]
        rgb_cnt = 0
        while not self.shutdown:
            # This cycle is executed once per time interval (50µs)

            # The servo has no cinematic, write the position directly
            buff_elem = 0
            #buff_elem=(self.__target_position[2] & 0b11)<<4
            #self.__current_position[2] = self.__target_position[2]

            # Check if the motors should brake
            brake = self.__brake()

            for i in [0,1]:
                # Check each axis independently
                if self.__target_position[i] != self.__current_position[i]:
                    if brake[i]:
                        self.__current_speed[i] -= +self.__max_acc[i] if (self.__current_speed[i] > 0) else -self.__max_acc[i]
                    else:
                        self.__current_speed[i] += +self.__max_acc[i] if (self.__target_position[i] > self.__current_position[i]) else -self.__max_acc[i]
                        if (self.__current_speed[i] > self.__top_speed[i]):
                            self.__current_speed[i] = self.__top_speed[i]
                        elif (self.__current_speed[i] < -self.__top_speed[i]):
                            self.__current_speed[i] = -self.__top_speed[i]

                    self.__internal_position[i] += self.__current_speed[i]

                    x = int(self.__internal_position[i]) - self.__current_position[i]
                    if abs(x) > 1:
                        print("Too fast: can't track position properly")

                    if x > 0:
                        self.__current_position[i] += 1
                        buff_elem |= 0b10 << (2*i)
                    elif x < 0:
                        self.__current_position[i] -= 1
                        buff_elem |= 0b11 << (2*i)

            if [self.__target_position[i] == self.__current_position[i] for i in [0,1]] == [True,True]:
                with self.__arrived_cv:
                    self.__arrived = True
                    self.__arrived_cv.notify_all()

            if self.__current_speed[0] > 0.2*self.__top_speed[0] or self.__current_speed[1] > 0.2*self.__top_speed[1]:
                servo_target = 60
            else:
                servo_target = self.__target_position[2]

            if rgb_cnt == 0:
                buff_elem |= 0b10000000
                # Update servo internal position & current position
                if servo_target > self.__internal_position[2]:
                    self.__internal_position[2] += self.__top_speed[2]
                    if servo_target < self.__internal_position[2]:
                        self.__internal_position[2] = servo_target
                elif servo_target < self.__internal_position[2]:
                    self.__internal_position[2] -= self.__top_speed[2]
                    if servo_target > self.__internal_position[2]:
                        self.__internal_position[2] = servo_target
                self.__current_position[2] = int(self.__internal_position[2])
                self.__rgb = self.rgb


            buff_elem |= ((self.__rgb[rgb_cnt//8] >> (7 - (rgb_cnt % 8))) & 1) << 6
            if rgb_cnt < 8:
                buff_elem |= ((self.__current_position[2] >> (7 - rgb_cnt)) & 1) << 5


            buff.append(buff_elem)

            rgb_cnt += 1
            if rgb_cnt >= 3*8:
                rgb_cnt = 0

            if (self.shutdown or len(buff) >= 200):
                self.ser.write(buff)
                buff = []

    def close(self):
        self.raw_goto(0,0,50)
        while self.__current_position != self.__target_position:
            time.sleep(0.1)
        self.shutdown = True
        self.worker.join()
        self.ser.close()

    def raw_goto(self, angle, radius, servo, sync = False):
        if angle < self.ranges[0][0]:
            angle = self.ranges[0][0]
        elif angle > self.ranges[0][1]:
            angle = self.ranges[0][1]

        if radius < self.ranges[1][0]:
            radius = self.ranges[1][0]
        elif radius > self.ranges[1][1]:
            radius = self.ranges[1][1]
        self.__target_position = [angle,radius,servo]
        with self.__arrived_cv:
            self.__arrived = False
            if sync:
                self.__arrived_cv.wait()

    def goto(self, angle, radius, servo, sync = False):
        self.raw_goto(int(angle * self.step_per_unit[0]), int(radius * self.step_per_unit[1]), servo, sync)

    def goto_xy(self, x, y, servo, sync = False):
        #Rotate 90°
        xx = y
        yy = -x

        radius, angle = self.__cart2pol(xx,yy)
        self.goto(angle, radius, servo, sync)

    def set_servo_speed(self, speed):
        self.__top_speed[2] = speed

    def servo_goto(self, pos):
        self.__target_position[2] = pos

hexi = Hexibe()
hexi.open("/dev/tty.usbmodem144401")


mp_drawing = mp.solutions.drawing_utils
mp_drawing_styles = mp.solutions.drawing_styles
mp_hands = mp.solutions.hands

ma = MovingAverageXY(30)

led_flash_cnt = 0

# For webcam input:
cap = cv2.VideoCapture(0)
with mp_hands.Hands(
    model_complexity=0,
    min_detection_confidence=0.5,
    min_tracking_confidence=0.5) as hands:
  while cap.isOpened():
    success, image = cap.read()
    if not success:
      print("Ignoring empty camera frame.")
      # If loading a video, use 'break' instead of 'continue'.
      continue

    # To improve performance, optionally mark the image as not writeable to
    # pass by reference.
    image.flags.writeable = False
    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    results = hands.process(image)

    # Draw the hand annotations on the image.
    image.flags.writeable = True
    image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    if results.multi_hand_landmarks:
        if led_flash_cnt > 0:
            #if (led_flash_cnt // 1) % 2 == 0:
            #    hexi.rgb = [0,0,0]
            #else:
            hexi.rgb = [25,10,25]
            led_flash_cnt = 0
        #hexi.rgb = [150,0,150]
        hexi.set_servo_speed(10)
        hand_landmarks=results.multi_hand_landmarks[0]
        #print(
        #  f'Index finger tip coordinates: (',
        #  f'{hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_TIP].x}, '
        #  f'{hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_TIP].y})'
        #)
        xx = int((0.5-hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_TIP].x)*200*2)
        yy = int((0.5-hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_TIP].y)*200*2)

        pos = ma.push([xx,yy])
        #print(xx,yy)
        hexi.goto_xy(xx,yy,120, False)
        mp_drawing.draw_landmarks(
            image,
            hand_landmarks,
            mp_hands.HAND_CONNECTIONS,
            mp_drawing_styles.get_default_hand_landmarks_style(),
            mp_drawing_styles.get_default_hand_connections_style())
    else:
        if led_flash_cnt < 20:
            #if (led_flash_cnt // 1) % 2 == 0:
            #    hexi.rgb = [0,0,0]
            #else:
            hexi.rgb = [int(25 * (math.exp(-led_flash_cnt))),int(10 * (math.exp(-led_flash_cnt))),int(25 * (math.exp(-led_flash_cnt)))]
            led_flash_cnt += 1
        else:
            hexi.rgb = [0,0,0]
            hexi.set_servo_speed(0.1)
            hexi.servo_goto(40)


    # Flip the image horizontally for a selfie-view display.
    cv2.imshow('MediaPipe Hands', cv2.flip(image, 1))
    if cv2.waitKey(5) & 0xFF == 27:
      break
cap.release()


hexi.close()
