FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Asia/Seoul

# 기본 개발 도구 + 네트워크 + SSH
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    gdb \
    vim \
    curl \
    wget \
    net-tools \
    iputils-ping \
    software-properties-common \
    openssh-client \
    && rm -rf /var/lib/apt/lists/*

# ROS2 저장소 추가
RUN add-apt-repository universe

RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    | apt-key add -

RUN echo "deb http://packages.ros.org/ros2/ubuntu jammy main" \
    > /etc/apt/sources.list.d/ros2.list

# ROS2 Humble (가벼운 ros-base)
RUN apt-get update && apt-get install -y \
    ros-humble-ros-base \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# ROS2 환경 자동 source
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

WORKDIR /drone_space
CMD ["/bin/bash"]

