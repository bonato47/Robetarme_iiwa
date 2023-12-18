# Handle ros distro
ARG ROS_DISTRO=noetic

FROM ghcr.io/aica-technology/ros-ws:${ROS_DISTRO} as ros-ws

# User provided arguments
ARG HOST_GID=1000
ARG GIT_NAME=""
ARG GIT_EMAIL=""
ARG USE_SIMD=OFF

# Tell docker we want to use bash instead of sh in general
SHELL ["/bin/bash", "-c"]

# Update cmake to a specific version
ARG CMAKE_VERSION="3.22.6"
WORKDIR /home/${USER}/
RUN sudo apt install build-essential libssl-dev
RUN wget https://github.com/Kitware/CMake/releases/download/v${CMAKE_VERSION}/cmake-${CMAKE_VERSION}.tar.gz
RUN tar -zxvf cmake-${CMAKE_VERSION}.tar.gz
WORKDIR /home/${USER}/cmake-${CMAKE_VERSION}
RUN ./bootstrap
RUN make -j .
RUN sudo make -j . install

### Add the user to the current GID of the host to avoid permisson issues in volumes
# AICA uses the same name for user and user group
ENV USER_GROUP=${USER}
WORKDIR /home/${USER}/
USER root
RUN if [ "HOST_GID" != "1000"] ; \
    then groupadd --gid ${HOST_GID} host_group && \
    usermod ${USER} -g ${HOST_GID} && \
    usermod ${USER} -a -G ${USER_GROUP}; fi
USER ${USER}

# Setup python version for noetic
RUN sudo apt update
RUN if [ "${ROS_DISTRO}" == "noetic" ] ; \
    then sudo apt install python-is-python3 ; fi

### Add a few tools
RUN sudo apt update && sudo apt install -y \
    bash-completion \
    silversearcher-ag \
    apt-transport-https \
    less \
    alsa-utils \
    ros-${ROS_DISTRO}-vrpn-client-ros \
    ros-${ROS_DISTRO}-ros-control \
    ros-${ROS_DISTRO}-ros-controllers \
    ros-${ROS_DISTRO}-moveit \
    && sudo apt upgrade -y && sudo apt clean


FROM ros-ws as inverse-kinematics

# Install qpPOASES
RUN git clone https://github.com/coin-or/qpOASES.git
RUN  cd qpOASES&& mkdir build && cd build \
    && cmake .. && make -j && sudo make install
RUN rm qpOASES -r

# Install pinochio and control libraire from epfl-lasa#
WORKDIR /home/${USER}
RUN git clone https://github.com/epfl-lasa/control-libraries.git --branch v6.3.1  --single-branch
WORKDIR /home/${USER}/control-libraries/source
RUN sudo bash install.sh -y

# Install relaxed ik
RUN curl -sSL https://bootstrap.pypa.io/get-pip.py -o get-pip.py
RUN sudo apt install -y ros-${ROS_DISTRO}-kdl-parser ros-${ROS_DISTRO}-kdl-parser-py
RUN python get-pip.py
RUN rm get-pip.py
RUN pip install readchar python-fcl scipy PyYaml matplotlib scipy tf
RUN pip install --upgrade numpy

# Need to be root to use ssh inside docker build
USER root
WORKDIR /home/${USER}/ros_ws/src
RUN --mount=type=ssh git clone git@github.com:lmunier/relaxed_ik_ros1.git
WORKDIR /home/${USER}/ros_ws/src/relaxed_ik_ros1
RUN git submodule init
RUN --mount=type=ssh git submodule update

# Transfer repo back to original user after root clone
WORKDIR /home/${USER}/ros_ws/src
RUN chown -R ${USER}:${HOST_GID} relaxed_ik_ros1
USER ${USER}

# Install rust dependency
RUN sudo apt install -y build-essential cmake
RUN sudo chmod 777 ~/.bashrc
RUN curl --proto '=https' --tlsv1.3 https://sh.rustup.rs -sSf | sh -s -- -y
RUN sudo chmod 644 ~/.bashrc
ENV PATH="/home/${USER}/.cargo/bin:${PATH}"

WORKDIR /home/${USER}/ros_ws/src/relaxed_ik_ros1/relaxed_ik_core

# Build relaxed ik
RUN cargo build
RUN cargo fix --lib -p relaxed_ik_core --allow-dirty

# Install gazebo (9 or 11 depending on distro)
WORKDIR /home/${USER}
RUN sudo apt update
RUN if [ "$ROS_DISTRO" = "noetic" ] ; then sudo apt install -y gazebo11 ; fi
RUN if [ "$ROS_DISTRO" = "melodic" ] ; then sudo apt install -y gazebo9 ; fi

# Install gazebo ros packages
RUN sudo apt install -y ros-${ROS_DISTRO}-gazebo-ros-pkgs ros-${ROS_DISTRO}-gazebo-ros-control

# Add f/t package for optitrack
RUN git clone https://github.com/epfl-lasa/net-ft-ros.git


FROM inverse-kinematics as iiwa-dependencies

# Install trak_ik_lib
RUN sudo apt install -y ros-${ROS_DISTRO}-trac-ik
RUN source /home/${USER}/.bashrc && rosdep update &&  rosdep install -y trac_ik_lib

# Handle SIMD option
RUN if [ "${USE_SIMD}" = "ON" ] ; \
 
RUN rm -rf /tmp/*

RUN sudo apt install -y \
    python3-vcstool \
    ros-noetic-rqt-joint-trajectory-controller \
    ros-noetic-moveit-commander \
    ros-noetic-teleop-twist-keyboard



FROM  iiwa-dependencies as finalisation

# Give bashrc back to user
WORKDIR /home/${USER}
RUN sudo chown -R ${USER}:${HOST_GID} .bashrc

# Add cmake option to bash rc if needed
RUN if [ "${USE_SIMD}" = "ON" ] ; \
    then echo "export ENABLE_SIMD=ON" >> /home/${USER}/.bashrc ; fi


FROM  finalisation as catkin-make

### Build ros workspace
WORKDIR /home/${USER}/ros_ws
RUN source /home/${USER}/.bashrc && rosdep install --from-paths src --ignore-src -r -y

RUN source /home/${USER}/.bashrc && catkin_make;

### Final apt clean
RUN sudo apt update && sudo apt upgrade -y && sudo apt clean


