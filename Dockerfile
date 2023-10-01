FROM ros:noetic-ros-base-focal

# NON ROOT USER ACCESS
ARG USERNAME=uzer
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Create the user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    #
    # [Optional] Add sudo support. Omit if you don't need to install software after connecting.
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

RUN apt-get update && apt-get install -y --no-install-recommends \
    git \  
    python3 \ 
    python3-pip \
    g++ \ 
    libgl1 \ 
    libglib2.0-0 \  
    libsm6 \ 
    libxext6 \ 
    net-tools \
    ros-$ROS_DISTRO-rviz

COPY requirements.txt requirements.txt
RUN pip install -r requirements.txt

USER $USERNAME
# Pepper robot dependencies
RUN mkdir -p /home/$USERNAME/catkin_ws/src && \
    cd /home/$USERNAME/catkin_ws/src && \
    git clone https://github.com/ros-naoqi/naoqi_driver.git && \
    git clone https://github.com/ros-naoqi/pepper_meshes.git


RUN cd /home/$USERNAME/catkin_ws/src && \
    rosdep update && \
    rosdep install -i -y --from-paths ./naoqi_driver

RUN cd /home/uzer/catkin_ws && /bin/bash -c "source /opt/ros/$ROS_DISTRO/setup.sh && catkin_make"

RUN echo "source /opt/ros/$ROS_DISTRO/setup.sh" >> "/home/$USERNAME/.bashrc"