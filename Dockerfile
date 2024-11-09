FROM ros:noetic-ros-base
COPY requirements.txt .
RUN apt-get -y update && apt-get install -y python-is-python3 python3-pip
RUN apt-get install -y ros-noetic-tf
RUN apt-get install -y ros-noetic-tf2-geometry-msgs 
RUN pip install -r requirements.txt && rm requirements.txt 
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc 
RUN apt-get install -y python3-catkin-tools python3-pip python-is-python3 
RUN apt-get install -y dos2unix 
RUN echo "source /root/Humanoid/catkin_ws/devel/setup.bash" >> ~/.bashrc 
RUN apt-get install -y ros-noetic-rosserial-arduino 
RUN apt-get install -y ros-noetic-rosserial
RUN pip3 install numpy-quaternion
# RUN pip3 install stable-baselines3[extra]
# RUN pip3 install tensorflow
RUN apt-get install -y tmux
RUN pip3 install pyzmq
RUN mkdir -p /etc/udev/rules.d/
RUN echo 'SUBSYSTEM=="tty", ATTRS{idVendor}=="2639", ATTRS{idProduct}=="0300", MODE=="0666"' > /etc/udev/rules.d/99-usb-serial.rules