FROM ros:humble

# Install python3 libraries
COPY requirements.txt /tmp
RUN pip3 install -r /tmp/requirements.txt

COPY src /opt/root_ws/src

WORKDIR /opt/root_ws

CMD [ "source", "/opt/ros/humble/install/setup.sh" ]
CMD [ "python3", "/opt/root_ws/src/send_images.py" ]