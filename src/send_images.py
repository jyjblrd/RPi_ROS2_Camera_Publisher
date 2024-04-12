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

    def __init__(self, name, width, height, compression):
        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.compression = compression

        self.q_raw = Queue.Queue()
        self.q_proc = Queue.Queue()
        t_reader = threading.Thread(target=self._reader)
        t_proc = threading.Thread(target=self._proc)

        t_reader.daemon = True
        t_reader.start()
        t_proc.daemon = True
        t_proc.start()


    # read frames as soon as they are available, keeping only most recent one
    def _reader(self):
        while True:
            ret, frame = self.cap.read()
            if not ret:
                global is_frame
                is_frame = False
                break
            if not self.q_raw.empty():
                try:
                   self.q_raw.get_nowait()   # discard previous (unprocessed) frame
                except Queue.Empty:
                    pass
            self.q_raw.put(frame)

    def _proc(self):
        while True:
            frame = self.q_raw.get()

            frame = cv2.rotate(frame, cv2.ROTATE_180)

            # compress
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), self.compression]
            result, encoded_image = cv2.imencode('.jpg', frame, encode_param)

            if not self.q_proc.empty():
                try:
                   self.q_proc.get_nowait()   # discard previous (unprocessed) frame
                except Queue.Empty:
                    pass

            self.q_proc.put(encoded_image)

    def read(self):
        return self.q_proc.get()



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
        self.cap = VideoCaptureQ(vid_path, width, height, self.compression)

    def capture_image(self):
        if is_frame == False:
            print('no more frames')
            return
        try:
            encoded_data = self.cap.read()

        except Exception as e:
            print(e)
            return

        msg = CompressedImage()
        msg.format = "jpeg"
        msg.data = encoded_data.tobytes()
        self.publisher_.publish(msg)

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

    except KeyboardInterrupt:
        pass
    finally:
        camera_publisher_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()