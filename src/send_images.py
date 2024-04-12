import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import cv2
import os
import queue as Queue
import threading
import time

vid_path = 0

is_frame = True
class VideoCaptureQ:

    def __init__(self, name, width, height):
        self.cap = cv2.VideoCapture(name)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

        self.q = Queue.Queue()
        t = threading.Thread(target=self._reader)
        t.daemon = True
        t.start()

    # read frames as soon as they are available, keeping only most recent one
    def _reader(self):
        while True:
            ret, frame = self.cap.read()
            if not ret:
                global is_frame
                is_frame = False
                break
            if not self.q.empty():
                try:
                    self.q.get_nowait()   # discard previous (unprocessed) frame
                except Queue.Empty:
                    pass
            self.q.put(frame)

    def read(self):
        return self.q.get()
    


class CameraPublisherNode(Node):
    def __init__(self):
        node_name = os.environ.get('NODE_NAME', 'camera_publisher_node')
        self.fps = int(os.environ.get('FPS', 30))
        width = int(os.environ.get('WIDTH', 1280))
        height = int(os.environ.get('HEIGHT', 720))
        self.compression = int(os.environ.get('COMPRESSION', 30))

        super().__init__(node_name)
        self.publisher_ = self.create_publisher(CompressedImage, f'{node_name}/camera_image/compressed', 10)

        # Initialize camera capture
        self.cap = VideoCaptureQ(vid_path, width, height)

    def capture_image(self):
        if is_frame == False:
            print('no more frames')
            return
        try:
            frame = self.cap.read()
        except Exception as e:
            print(e)
            return

        frame = cv2.rotate(frame, cv2.ROTATE_180)

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

def main(args=None):
    rclpy.init(args=args)
    camera_publisher_node = CameraPublisherNode()
    last_step_time = 0

    try:
        while rclpy.ok():
            if time.time() - last_step_time > 1/camera_publisher_node.fps:
                last_step_time = time.time()
                camera_publisher_node.capture_image()

            rclpy.spin_once(camera_publisher_node, timeout_sec=0)
            time.sleep(0.0001)
    except KeyboardInterrupt:
        pass
    finally:
        camera_publisher_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
