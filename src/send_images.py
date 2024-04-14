import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, CameraInfo
import cv2
import os
import queue as Queue
import threading
import time
import json
from geometry_msgs.msg import TransformStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster



vid_path = 0

is_frame = True
class VideoCaptureQ:

    def __init__(self, name, width, height, compression, node):
        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.compression = compression
        self.node = node

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

            stamp = self.node.get_clock().now().to_msg()
            
            self.q_raw.put((stamp, frame))

    def _proc(self):
        while True:
            (stamp, frame) = self.q_raw.get()

            frame = cv2.rotate(frame, cv2.ROTATE_180)

            # compress
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), self.compression]
            result, encoded_image = cv2.imencode('.jpg', frame, encode_param)

            if not self.q_proc.empty():
                try:
                   self.q_proc.get_nowait()   # discard previous (unprocessed) frame
                except Queue.Empty:
                    pass

            self.q_proc.put((stamp, encoded_image))

    def read(self):
        return self.q_proc.get()



class CameraPublisherNode(Node):
    def __init__(self):
        node_name = os.environ.get('NODE_NAME', 'camera_publisher_node')
        self.fps = int(os.environ.get('FPS', 30))
        width = int(os.environ.get('WIDTH', 1280))
        height = int(os.environ.get('HEIGHT', 720))
        self.compression = int(os.environ.get('COMPRESSION', 30))
        self.node_name = node_name

        super().__init__(node_name)
        self.image_publisher = self.create_publisher(CompressedImage, f'{node_name}/camera_image/compressed', 10)
        self.camera_info_publisher = self.create_publisher(CameraInfo, f'{node_name}/camera_image/camera_info', 10)
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)

        with open("/opt/root_ws/host_files/camera_info.json") as f:
            camera_info = json.load(f)
        self.camera_info_msg = CameraInfo()
        self.camera_info_msg.header.frame_id = f"{node_name}_corrected"
        self.camera_info_msg.header.stamp = self.get_clock().now().to_msg()
        self.camera_info_msg.height = camera_info["height"]
        self.camera_info_msg.width = camera_info["width"]
        self.camera_info_msg.k = camera_info["camera_matrix"]
        self.camera_info_msg.d = camera_info["distortion_coefficients"]
        self.camera_info_msg.r = camera_info["rectification_matrix"]
        self.camera_info_msg.p = camera_info["projection_matrix"]
        self.camera_info_msg.distortion_model = camera_info["distortion_model"]

        with open("/opt/root_ws/host_files/mocap_calibration.json") as f:
            mocap_calibration = json.load(f)
        self.correction_transform = TransformStamped()
        self.correction_transform.header.stamp = self.get_clock().now().to_msg()
        self.correction_transform.header.frame_id = self.node_name
        self.correction_transform.child_frame_id = f"{self.node_name}_corrected"
        self.correction_transform.transform.translation.x = mocap_calibration["translation"]["x"]
        self.correction_transform.transform.translation.y = mocap_calibration["translation"]["y"]
        self.correction_transform.transform.translation.z = mocap_calibration["translation"]["z"]
        self.correction_transform.transform.rotation.x = mocap_calibration["rotation"]["x"]
        self.correction_transform.transform.rotation.y = mocap_calibration["rotation"]["y"]
        self.correction_transform.transform.rotation.z = mocap_calibration["rotation"]["z"]
        self.correction_transform.transform.rotation.w = mocap_calibration["rotation"]["w"]

        # Initialize camera capture
        self.cap = VideoCaptureQ(vid_path, width, height, self.compression, self)

    def capture_image(self):
        if is_frame == False:
            print('no more frames')
            return
        try:
            (stamp, encoded_data) = self.cap.read()

        except Exception as e:
            print(e)
            return

        msg = CompressedImage()
        msg.header.frame_id = f"{self.node_name}_corrected"
        msg.header.stamp = stamp
        msg.format = "jpeg"
        msg.data = encoded_data.tobytes()
        self.image_publisher.publish(msg)

    def publish_camera_info(self):
        self.camera_info_publisher.publish(self.camera_info_msg)

    def publish_corrected_tf(self):
        self.tf_static_broadcaster.sendTransform(self.correction_transform)

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
                camera_publisher_node.publish_corrected_tf()

            rclpy.spin_once(camera_publisher_node, timeout_sec=0)

    except KeyboardInterrupt:
        pass
    finally:
        camera_publisher_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()