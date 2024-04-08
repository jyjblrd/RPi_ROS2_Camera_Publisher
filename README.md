Raspberry Pi has to use legacy camera driver. Edit `/boot/firmware/config.txt`: delete `auto_detect_camera=1` and add `start_x=1`.

Build Container: `sudo docker build https://github.com/jyjblrd/RPi_ROS2_Camera_Publisher.git -t rpi_ros2_camera_publisher`

Run Container: `sudo docker run -d --name my_container -e NODE_NAME='rpi_cam_0' -e COMPRESSION='30' -e WIDTH='1240' -e HEIGHT='720' -e FPS='30' --net=host --privileged rpi_ros2_camera_publisher`