import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from rclpy.qos import QoSProfile, QoSHistoryPolicy
import socket
import numpy as np
from cv_bridge import CvBridge
import cv2 

class ImageUDPServer(Node):
    def __init__(self):
        super().__init__('image_udp_server')
        
        # Define QoS profile for BestEffort
        qos_profile = QoSProfile(depth=2)
        qos_profile.history = QoSHistoryPolicy.KEEP_LAST

        self.publisher_ = self.create_publisher(Image, 'received/image_raw', qos_profile)
        self.bridge = CvBridge()
        self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_socket.bind(('127.0.0.1', 55005))
        self.get_logger().info('Listening for UDP images...')

        # Start a timer to receive images
        self.timer = self.create_timer(0.1, self.receive_image)

    def receive_image(self):
        try:
            data, _ = self.udp_socket.recvfrom(65507)
            np_arr = np.frombuffer(data, np.uint8)
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

            if frame is not None:
                # Show the received image in a window
                # cv2.imshow("Received Image", frame)
                # cv2.waitKey(1)  # Add a short delay to update the window

                image_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
                self.publisher_.publish(image_msg)
                # self.get_logger().info('Published received image to ROS topic')
            else:
                self.get_logger().warn('Received frame is None')

        except Exception as e:
            self.get_logger().error(f'Error receiving image: {e}')

def main(args=None):
    rclpy.init(args=args)
    server = ImageUDPServer()
    rclpy.spin(server)
    server.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()  # Ensure all OpenCV windows are closed

if __name__ == '__main__':
    main()
