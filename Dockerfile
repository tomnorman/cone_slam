# This is a Dockerfile for osrf/sros:kinetic with SROS
FROM ros:kinetic-robot-xenial

# install packages
RUN apt-get update && \
    apt-get install --no-install-recommends -y \
    apt-utils \
    ros-kinetic-desktop=1.3.2-0* \
    libeigen3-dev \
    libgl1-mesa-dev \
    libglew-dev \
    python-catkin-tools \
    python-pip && \
    rm -rf /var/lib/apt/lists/*

RUN echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc

# install bootstrap
#   and dev tools
RUN pip install numpy
RUN pip install scipy==0.16
RUN pip install scikit-learn==0.20
    

# set env and ws
ENV BASH_ENV /root/.bashrc
ENV ROS_DISTRO kinetic
ENV CATKIN_WS=/root/formula_ws
ENV PATH="/opt/ros/kinetic/bin:${PATH}"
RUN mkdir -p $CATKIN_WS/src

# download sourcecode for pangolin
WORKDIR $CATKIN_WS/src
RUN git clone https://github.com/stevenlovegrove/Pangolin.git
#RUN cd Pangolin && \
#    mkdir build && \
#    cd build && \
#    cmake .. && \
#    cmake --build .

# download sourcecode for cone_slam
RUN git clone https://github.com/yardenshap/custom_msgs.git
RUN git clone https://tomnorman:formulalab123@github.com/tomnorman/cone_slam.git
RUN git clone https://github.com/AlexeyAB/darknet.git
RUN cd darknet && \
    sed -i 's/LIBSO=0/LIBSO=1/g' Makefile && \
    make && \
    mkdir ../cone_slam/orb_slam2/Thirdparty/darknet/lib && \
    mv libdarknet.so ../cone_slam/orb_slam2/Thirdparty/darknet/lib/libdarklib.so
WORKDIR $CATKIN_WS/src/cone_slam/orb_slam2/
RUN ./build.sh
RUN echo "source $CATKIN_WS/devel/setup.bash" >> ~/.bashrc
WORKDIR $CATKIN_WS

#sudo docker build --tag=slam .
#sudo docker run -it slam
#catkin build
#source ~/.bashrc