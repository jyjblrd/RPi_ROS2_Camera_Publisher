FROM ros:humble

COPY src /opt/root_ws/src

WORKDIR /opt/root_ws

CMD [ "source", "/opt/ros/humble/install/setup.sh" ]
CMD [ "python3", "/opt/root_ws/src/send_images.py" ]