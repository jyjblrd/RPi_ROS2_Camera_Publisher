Raspberry Pi has to use legacy camera driver. Edit `/boot/firmware/config.txt`: delete `auto_detect_camera=1` and add `start_x=1`.

Build Container: `sudo docker build https://github.com/jyjblrd/RPi_ROS2_Camera_Publisher.git -t rpi_ros2_camera_publisher`

Pull Prebuilt Container (linux/arm64): `sudo docker pull joshuabird/rpi_ros2_camera_publisher:latest`

Run Container: `sudo docker run -d --rm --name camera_publisher -e NODE_NAME='rpi_cam_0' -e COMPRESSION='30' -e WIDTH='1240' -e HEIGHT='720' -e FPS='30' --net=host --privileged joshuabird/rpi_ros2_camera_publisher:latest`