# This node uses the camera to recognize and interpret traffic signals
# and works with nav2_node to obey them.
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

# TrafficLight.msg used to notify what signal has been detected and how far it is
# string signal_type  # The type of signal detected (e.g., "RED", "GREEN", "YELLOW")
# float64 distance    # The estimated distance to the signal in meters

class TrafficLightRecognitionNode(Node):
    def __init__(self):
        super().__init__('traffic_light_recognition_node')

        # subscribe to the camera
        self.subscription = self.create_subscription(
            Image,
            'camera_topic',
            self.image_callback,
            10)
        self.subscription  # prevent unused variable warning

        self.publisher = self.create_publisher(TrafficLight, 'traffic_light_topic', 10)
        self.subscription = self.create_subscription(
            Image,
            'camera_topic',
            self.image_callback,
            10)
        self.subscription  # prevent unused variable warning

    def image_callback(self, msg):
        # Logic for recognizing traffic lights and signals in the image
        # Uses OpenCV and potentially machine-learning libraries
        traffic_light_msg = TrafficLight()
        traffic_light_msg.signal_type = 'RED'  # replace with actual detected signal
        traffic_light_msg.distance = 10.0  # replace with actual estimated distance
        self.publisher.publish(traffic_light_msg)
        self.get_logger().info('Image received and processed')

def main(args=None):
    rclpy.init(args=args)

    traffic_light_recognition_node = TrafficLightRecognitionNode()

    rclpy.spin(traffic_light_recognition_node)

    traffic_light_recognition_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()