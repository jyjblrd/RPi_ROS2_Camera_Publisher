import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import cv2
import os

class CameraPublisherNode(Node):
    def __init__(self):
        node_name = os.environ.get('NODE_NAME', 'camera_publisher_node')
        fps = os.environ.get('FPS', 30)
        width = os.environ.get('WIDTH', 1280)
        height = os.environ.get('HEIGHT', 720)
        self.compression = os.environ.get('COMPRESSION', 30)

        super().__init__(node_name)
        self.publisher_ = self.create_publisher(CompressedImage, f'{node_name}/camera_image/compressed', 10)
        timer_period = 1/fps
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Initialize camera capture
        self.capture = cv2.VideoCapture(0)
        self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

    def timer_callback(self):
        ret, frame = self.capture.read()
        if ret:
            # Compress the image
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), self.compression]
            result, encoded_image = cv2.imencode('.jpg', frame, encode_param)

            if result:
                msg = CompressedImage()
                msg.format = "jpeg"
                msg.data = encoded_image.tobytes()
                self.publisher_.publish(msg)
            else:
                self.get_logger().error("Failed to compress image")
        else:
            self.get_logger().error("Failed to capture frame from camera")

def main(args=None):
    rclpy.init(args=args)
    camera_publisher_node = CameraPublisherNode()
    rclpy.spin(camera_publisher_node)
    camera_publisher_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
