import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from rclpy.qos import QoSProfile, QoSHistoryPolicy
import socket
import numpy as np
from cv_bridge import CvBridge
import cv2 
import struct 

class ImageUDPServer(Node):
    def __init__(self):
        super().__init__('image_udp_server')
        
        # Define QoS profile for BestEffort
        qos_profile = QoSProfile(depth=2)
        qos_profile.history = QoSHistoryPolicy.KEEP_LAST

        self.publisher_ = self.create_publisher(Image, 'received/image_raw', qos_profile)
        self.bridge = CvBridge()

        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.client_socket.connect(('127.0.0.1', 55005))
        self.get_logger().info('Listening for UDP images...')

        # Start a timer to receive images
        self.timer = self.create_timer(0.01, self.receive_image)

    def recv_exact(self, size):
        data = b''  # Start with an empty bytes object
        while len(data) < size:
            chunk = self.client_socket.recv(size - len(data))  # Receive the remaining bytes
            if not chunk:
                raise ConnectionError("Socket connection lost")
            data += chunk  # Append the chunk to the data
        return data

    def receive_image(self):
        try:
            data = b""
            # Receive the header data (height, width, and size of image)
            header = self.recv_exact(12+4+1)
            height, width, size, step, big_endian = struct.unpack(">IIIIB", header)

            data = self.recv_exact(size)

            frame = np.frombuffer(data, np.uint8)

            # cv2.imshow("Received Image", frame)
            # cv2.waitKey(1)

            if frame is not None:
                # Show the received image in a window
                # cv2.imshow("Received Image", frame)
                # cv2.waitKey(1)  # Add a short delay to update the window

                # Create the Image message
                image_msg = Image()
                image_msg.header = Header()
                image_msg.header.stamp = self.get_clock().now().to_msg()
                image_msg.header.frame_id = 'camera_frame'
                
                # Set image metadata
                image_msg.height = height
                image_msg.width = width
                image_msg.encoding = 'rgb8'
                image_msg.is_bigendian = big_endian
                image_msg.step = step  # Assuming 3 bytes per pixel for RGB8
                
                # self.get_logger().info('data size: ' + str(len(data)))

                # Fill image data
                image_msg.data = list(data)
                
                # self.get_logger().info('data size2: ' + str(len(image_msg.data)))

                # Publish the message
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
