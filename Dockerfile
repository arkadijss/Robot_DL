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
    net-tools

RUN apt-get update && apt-get install -y \
    ros-$ROS_DISTRO-rviz \
    ros-$ROS_DISTRO-xacro \
    ros-$ROS_DISTRO-moveit-ros-visualization 

# Pepper gazebo dependencies
RUN apt-get update && apt-get install -y \
    ros-$ROS_DISTRO-gazebo-ros \ 
    ros-$ROS_DISTRO-gazebo-ros-pkgs \
    ros-$ROS_DISTRO-gazebo-ros-control \
    ros-$ROS_DISTRO-effort-controllers \
    ros-$ROS_DISTRO-joint-state-controller

USER $USERNAME
# Pepper robot dependencies
RUN mkdir -p /home/$USERNAME/catkin_ws/src && \
    cd /home/$USERNAME/catkin_ws/src && \
    # bridge
    git clone https://github.com/ros-naoqi/naoqi_driver.git && \
    # base - pepper_bringup, pepper_description, pepper_sensors
    git clone https://github.com/ros-naoqi/pepper_robot.git && \
    # virtual - control + gazebo
    git clone https://github.com/ros-naoqi/pepper_virtual.git && \
    # dcm bringup
    git clone https://github.com/ros-naoqi/pepper_dcm_robot.git && \
    # moveit for motion planning
    git clone https://github.com/ros-naoqi/pepper_moveit_config.git

    
RUN cd /home/$USERNAME/catkin_ws/src && \
    rosdep update && \
    rosdep install -i -y --from-paths ./naoqi_driver

RUN cd /home/uzer/catkin_ws && /bin/bash -c "source /opt/ros/$ROS_DISTRO/setup.sh && catkin_make"

RUN echo "source /opt/ros/$ROS_DISTRO/setup.sh" >> "/home/$USERNAME/.bashrc"

COPY requirements.txt requirements.txt
RUN pip install -r requirements.txt