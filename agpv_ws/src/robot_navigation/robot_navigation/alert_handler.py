#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt16, String
import time

class AlertHandler(Node):
    def __init__(self):
        super().__init__('alert_handler');
        self.publisher_ = self.create_publisher(UInt16, '/beep', 10)
        self.subscription = self.create_subscription(
            String,
            '/station_status',
            self.listener_callback,
            10)
        self.current_station = "None"

        print('__init__ complete')

    def sound_alarm(self):
        msg = UInt16()
        for i in range(0, 5):
          msg.data = 1
          self.publisher_.publish(msg)
          time.sleep(0.5)
          msg.data = 0
          self.publisher_.publish(msg)
          time.sleep(0.5)

    def listener_callback(self, msg):
        station = msg.data.split(':')[0]
        print(station, msg.data)
        if not (station == self.current_station):
          self.current_station = station
          if "Error" in msg.data:
            self.sound_alarm()

def main(args = None):
    print('main')
    rclpy.init(args = args)
    alert_handler = AlertHandler()
    rclpy.spin(alert_handler)
    rclpy.shutdown()

if __name__ == '__main__':
    main()