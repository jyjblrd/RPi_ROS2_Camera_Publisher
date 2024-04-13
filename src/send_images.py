import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, CameraInfo
import cv2
import os
import queue as Queue
import threading
import time
import json


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
        self.image_publisher = self.create_publisher(CompressedImage, f'{node_name}/camera_image/compressed', 10)
        self.camera_info_publisher = self.create_publisher(CameraInfo, f'{node_name}/camera_image/camera_info', 10)

        with open("/opt/root_ws/host_files/camera_info.json") as f:
            calib_data = json.load(f)
        self.camera_info_msg = CameraInfo()
        self.camera_info_msg.header.frame_id = node_name
        self.camera_info_msg.height = calib_data["height"]
        self.camera_info_msg.width = calib_data["width"]
        self.camera_info_msg.k = calib_data["camera_matrix"]
        self.camera_info_msg.d = calib_data["distortion_coefficients"]
        self.camera_info_msg.r = calib_data["rectification_matrix"]
        self.camera_info_msg.p = calib_data["projection_matrix"]
        self.camera_info_msg.distortion_model = calib_data["distortion_model"]

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
        self.image_publisher.publish(msg)

    def publish_camera_info(self):
        self.camera_info_publisher.publish(self.camera_info_msg)

def main(args=None):
    rclpy.init(args=args)
    camera_publisher_node = CameraPublisherNode()
    last_step_time = 0

    try:
        while rclpy.ok():
            if time.time() - last_step_time > 1/camera_publisher_node.fps:
                last_step_time = time.time()
                camera_publisher_node.capture_image()
                camera_publisher_node.publish_camera_info()

            rclpy.spin_once(camera_publisher_node, timeout_sec=0)

    except KeyboardInterrupt:
        pass
    finally:
        camera_publisher_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()