# Eu peguei esse dockerfile padrão da documentação https://docs.ros.org/en/humble/How-To-Guides/Setup-ROS-2-with-VSCode-and-Docker-Container.html

FROM ros:humble
ARG USERNAME=remy
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
RUN apt-get update && apt-get upgrade -y
RUN apt-get install -y python3-pip

RUN apt-get update && apt-get upgrade -y \
    && apt-get install -y \
        python3-pip \
        build-essential \
        git \
        wget \
        curl \
        lsb-release \
        # ROS 2 tools
        ros-humble-rviz2 \
        ros-humble-gazebo-ros-pkgs \
        ros-humble-gazebo-ros2-control \
        ros-humble-slam-toolbox \
        ros-humble-navigation2 \
        ros-humble-nav2-bringup \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /dev_ws
RUN mkdir src

RUN echo "source /opt/ros/humble/setup.bash" >> /etc/bash.bashrc

# [Optional] Set the default user. Omit if you want to keep the default as root.
USER $USERNAME
CMD ["/bin/bash"]