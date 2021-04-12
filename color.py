import numpy as np
import cv2
import serial
import time
import imutils
class Utils:
    def ret_red_non_zero(self, frame):
        hsv_red = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower_red = np.array([0, 50, 50])
        upper_red = np.array([10, 255, 255])
        mask0_red = cv2.inRange(hsv_red, lower_red, upper_red)
        '''
        # upper mask (170-180)
        lower_red = np.array([170, 50, 50])
        upper_red = np.array([180, 255, 255])
        mask1_red = cv2.inRange(hsv_red, lower_red, upper_red)
        '''
        # join my masks
        mask_red = mask0_red #+ mask1_red
        # or your HSV image, which I *believe* is what you want
        output_hsv_red = hsv_red.copy()
        output_hsv_red[np.where(mask_red == 0)] = 0
        bgr_red = cv2.cvtColor(output_hsv_red, cv2.COLOR_HSV2BGR)
        gray_red = cv2.cvtColor(bgr_red, cv2.COLOR_BGR2GRAY)
        cv2.imshow("red",gray_red)
        non_zero_red = cv2.countNonZero(gray_red)

        return non_zero_red

    def ret_yellow_non_zero(self, frame):
        hsv_yellow = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower_yellow = np.array([21, 39, 64])
        upper_yellow = np.array([40, 255, 255])

        mask_yellow = cv2.inRange(hsv_yellow, lower_yellow, upper_yellow)

        output_hsv_yellow = hsv_yellow.copy()
        output_hsv_yellow[np.where(mask_yellow==0)] = 0
        bgr_yellow = cv2.cvtColor(output_hsv_yellow,cv2.COLOR_HSV2BGR)
        gray_yellow = cv2.cvtColor(bgr_yellow, cv2.COLOR_BGR2GRAY)
        cv2.imshow("yellow",gray_yellow)
        non_zero_yellow = cv2.countNonZero(gray_yellow)

        return non_zero_yellow

    def ret_black_non_zero(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray,(7,7),0)
        _, threshold = cv2.threshold(blur,15,255,cv2.THRESH_BINARY_INV)
        black_non_zero = cv2.countNonZero(threshold)

        return black_non_zero




cap = cv2.VideoCapture(1)

_, first_frame = cap.read()

u = Utils()
ser = serial.Serial(port='COM3', baudrate=9600, timeout=.1)
thresh = 40000
thresh_red = 100000
thresh_black = 20000
red_flag = 0
yellow_flag = 0
black_flag = 0
while True:
    _, frame = cap.read()
    cv2.imshow("frame", frame)
    red = u.ret_red_non_zero(frame)
    yellow = u.ret_yellow_non_zero(frame)
    black = u.ret_black_non_zero(frame)
    print("red", red, "yellow", yellow, "black", black)
    #print("yellow", yellow)
    if red >= thresh_red and red_flag==0:
        ser.write(b'0')
        print("red--------------------------------------------------------")
        red_flag = 1
        time.sleep(6)
    elif yellow >= thresh and yellow_flag==0:
        ser.write(b'1')
        print("yellow######################################################")
        yellow_flag = 1
        time.sleep(6)
    elif black >= thresh_black and black_flag==0:
        ser.write(b'2')
        print("black&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&")
        black_flag = 1
        time.sleep(6)
    elif (red < thresh_red and red_flag==1) or (yellow < thresh and yellow_flag==1) or (black< thresh_black and black_flag==1):
        red_flag=0
        yellow_flag = 0
        black_flag = 0
        ser.write(b'6')
        print("Faka")
    k = cv2.waitKey(1)
    if k == 27:
        break
'''
hsv_black = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower_black = np.array([0, 0, 64])
        upper_black = np.array([180, 255, 63])

        mask_black = cv2.inRange(hsv_black, lower_black, upper_black)

        output_hsv_black = hsv_black.copy()
        output_hsv_black[np.where(mask_black == 0)] = 0
        bgr_black = cv2.cvtColor(output_hsv_black, cv2.COLOR_HSV2BGR)
        gray_black = cv2.cvtColor(bgr_black, cv2.COLOR_BGR2GRAY)
        non_zero_black = cv2.countNonZero(gray_black)

        return non_zero_black
'''