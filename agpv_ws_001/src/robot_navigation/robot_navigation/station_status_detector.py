#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time
import cv2
import numpy as np
import pyzbar.pyzbar as pyzbar

class StationStatusDetector(Node):
    def __init__(self):
        super().__init__('station_status_detector');
        self.publisher_ = self.create_publisher(String, 'station_status', 10)
        self.timer = self.create_timer(0.2, self.timer_callback)
        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        print('__init__ complete')

    def publish_station_status(self, station_status):
        msg = String()
        msg.data = station_status
        self.publisher_.publish(msg)

    def detect_station_status(self):
        #print("detecting station status")
        _, frame = self.cap.read()
        #print('captured frame')
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        barCodeData = None
        barcodes = pyzbar.decode(gray)
        if barcodes:
            encoding = 'UTF-8'
            barcodeData = barcodes[0].data.decode(encoding)
            print(barcodeData)
        station_status = "Station 1 is OK"
        return True, station_status

    def timer_callback(self):
        station_status_detected, station_status = self.detect_station_status()
        if (station_status_detected):
          self.publish_station_status(station_status)

def main(args = None):
    print('main')
    rclpy.init(args = args)
    station_status_detector = StationStatusDetector()
    rclpy.spin(station_status_detector)
    station_status_detector.cap.release()
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
