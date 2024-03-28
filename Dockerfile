FROM osrf/ros:noetic-desktop
LABEL authors="chlee-rdv"
ARG DEBIAN_FRONTEND=noninteractive
# ----------------------------------------------------------------------------------------------
# 지역 설정
ENV TZ=Asia/Seoul
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

RUN apt-get update && apt-get install -y --no-install-recommends \
    vim git wget tar xz-utils \
    build-essential cmake g++
# ----------------------------------------------------------------------------------------------
# install PCL package
RUN apt-get install -y libpcl-dev
# ----------------------------------------------------------------------------------------------
# install armadillo package
WORKDIR /root/armadillo
RUN apt-get install -y libblas-dev liblapack-dev
RUN wget -O armadillo.tar.xz https://sourceforge.net/projects/arma/files/armadillo-12.8.1.tar.xz/download && \
    tar -xf armadillo.tar.xz --strip-components=1 && \
    rm armadillo.tar.xz
RUN cmake . && \
    cmake --build . --config Release && \
    cmake --install .
# ----------------------------------------------------------------------------------------------
# ROS 의존성 설치
COPY ./dependencies /tmp/dependencies
RUN apt-get update && xargs apt-get install -y
RUN apt-get install -y \
    ros-noetic-pcl-ros \
    ros-noetic-pcl-conversions \
	ros-noetic-tf2-sensor-msgs \
	ros-noetic-image-transport-plugins
# ----------------------------------------------------------------------------------------------
# APT 패키지 리스트 삭제(이미지 크기를 줄이기 위함)
RUN sudo apt-get clean && sudo rm -rf /var/lib/apt/lists/*
# ----------------------------------------------------------------------------------------------
ARG WORKSPACE="/root/catkin_ws"

#COPY ../lidar-camera-fusion ${WORKSPACE}/src
COPY . ${WORKSPACE}/src/lidar-camera-fusion
WORKDIR ${WORKSPACE}/src
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash; catkin_init_workspace"
WORKDIR ${WORKSPACE}
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash; catkin_make;"
RUN echo "source ${WORKSPACE}/devel/setup.bash" >> ~/.bashrc


CMD ["tail", "-f", "/dev/null"]
ADD entrypoint.sh /entrypoint.sh
ENTRYPOINT [ "/entrypoint.sh" ]
