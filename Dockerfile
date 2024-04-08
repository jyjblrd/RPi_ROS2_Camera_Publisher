FROM ros:humble

# Install python3 libraries
RUN sudo apt update && sudo apt install -y python3-pip

COPY requirements.txt /tmp
RUN pip3 install -r /tmp/requirements.txt

COPY src /opt/root_ws/src

WORKDIR /opt/root_ws

CMD [ "source", "/opt/ros/humble/install/setup.sh" ]
CMD [ "python3", "/opt/root_ws/src/send_images.py" ]