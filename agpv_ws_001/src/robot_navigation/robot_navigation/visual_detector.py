#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time
import cv2
import numpy as np
import pyzbar.pyzbar as pyzbar

class VisualDetector(Node):
    def __init__(self):
        super().__init__('visual_detector');
        self.publisher_station_status_ = self.create_publisher(String, 'station_status', 10)
        self.publisher_red_light_ = self.create_publisher(String, 'red_light', 10)
        self.last_time_detected = None
        self.last_published_state = False
        self.timer = self.create_timer(0.2, self.timer_callback)
        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        print('__init__ complete')

    def publish_station_status(self, station_status):
        msg = String()
        msg.data = station_status
        self.publisher_station_status_.publish(msg)

    def detect_station_status(self, frame):
        #print("detecting station status")
        #_, frame = self.cap.read()
        #print('captured frame')
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        barcodeData = None
        barcodes = pyzbar.decode(gray)
        if barcodes:
            encoding = 'UTF-8'
            barcodeData = barcodes[0].data.decode(encoding)
            #print(barcodeData)
            station_status = barcodeData
            return True, station_status
        else:
            return False, None

    def detect_red_light(self, frame):
        #_, frame = self.cap.read()
        # Convert BGR to HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Define range for red color in HSV
        # lower_red = np.array([160, 100, 100])
        # upper_red = np.array([180, 255, 255])
        # lower_red = np.array([160, 140, 230])
        # upper_red = np.array([180, 160, 255])
        # lower_red = np.array([160, 140, 230])
        # upper_red = np.array([180, 160, 255])
        lower_red = np.array([160, 100, 100])
        upper_red = np.array([179, 255, 255])


        # Threshold the HSV image to get only red colors
        mask = cv2.inRange(hsv, lower_red, upper_red)

        # Bitwise-AND mask and original image
        res = cv2.bitwise_and(frame, frame, mask=mask)

        # Blur the image to reduce noise
        blur = cv2.GaussianBlur(res, (15, 15), 0)

        # Convert BGR to grayscale
        gray = cv2.cvtColor(blur, cv2.COLOR_BGR2GRAY)

        # Apply Hough Circle Transform
        circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1, 20, param1=50, param2=30, minRadius=0, maxRadius=0)
        
        return circles is not None

    def publish_state(self, state):
        msg = String()
        msg.data = 'Red light is ' + ('on' if state else 'off')
        self.publisher_red_light_.publish(msg)
        self.last_published_state = state

    def timer_callback(self):
        _, frame = self.cap.read()
        station_status_detected, station_status = self.detect_station_status(frame)
        if (station_status_detected):
          self.publish_station_status(station_status)
        red_light_detected = self.detect_red_light(frame)
        if red_light_detected:
            #print('red light detected!')
            self.last_time_detected = time.time()
            if self.last_published_state is not True:
                self.publish_state(True)
        else:
            #print('red light not detected')
            if self.last_time_detected is not None \
                and ((time.time() - self.last_time_detected) > 1.0) \
                and self.last_published_state is not False:
                self.publish_state(False)

def main(args = None):
    print('main')
    rclpy.init(args = args)
    visual_detector = VisualDetector()
    rclpy.spin(visual_detector)
    visual_detector.cap.release()
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
