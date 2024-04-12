# RPi ROS2 Camera Publisher

### Raspberry Pi Config
Raspberry Pi has to use legacy camera driver. Edit `/boot/firmware/config.txt`: delete `auto_detect_camera=1` and add `start_x=1`.

### Pull and Build
Pull Prebuilt Container (linux/arm64): `sudo docker pull joshuabird/rpi_ros2_camera_publisher:latest`

Run Container: `sudo docker run -d --rm --name camera_publisher -e NODE_NAME='rpi_cam_0' -e COMPRESSION='30' -e WIDTH='1240' -e HEIGHT='720' -e FPS='30' --net=host --privileged joshuabird/rpi_ros2_camera_publisher:latest`

### Run On Startup
If you want the container to pull the latest version and run on startup, create a file called `/etc/systemd/system/docker-rpi-cam.service` and paste the following into it:
```
[Unit]
Description=Pull and run rpi cam docker image
Requires=docker.service
After=docker.service

[Service]
Restart=always
ExecStartPre=/usr/bin/docker pull joshuabird/rpi_ros2_camera_publisher:latest
ExecStart=/usr/bin/docker run --rm --privileged -e NODE_NAME='rpi_cam_0' -e COMPRESSION='30' -e WIDTH='1280' -e HEIGHT='720' -e FPS='30' -->
ExecStop=/usr/bin/docker stop joshuabird/rpi_ros2_camera_publisher:latest
TimeoutSec=900

[Install]
WantedBy=default.target
```

Then run `sudo systemctl enable docker-rpi-cam.service` then `sudo systemctl start docker-rpi-cam.service`.
It may take a few minutes to start the first time, as it needs to download the docker image.
