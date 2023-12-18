# Handle ros distro
ARG ROS_DISTRO=noetic

FROM ghcr.io/aica-technology/ros-ws:${ROS_DISTRO} as ros-ws

# User provided arguments
ARG HOST_GID=1000
ARG GIT_NAME=""
ARG GIT_EMAIL=""
ARG USE_SIMD=OFF
ARG NB_CPU_RESIZED=6
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
RUN make -j  ${NB_CPU_RESIZED} .
RUN sudo make -j  ${NB_CPU_RESIZED} . install

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
    && cmake .. && make -j  ${NB_CPU_RESIZED}&& sudo make install
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
    then export CMAKE_CXX_FLAGS="-march=native -faligned-new" ; fi

### Install all dependencies of IIWA ROS
# Clone KUKA FRI (need to be root to clone private repo)

# Setup git identity to publish patch on SIMD
RUN git config --global user.name "${GIT_NAME}"
RUN git config --global user.email "${GIT_EMAIL}"

WORKDIR /tmp
USER root
RUN --mount=type=ssh git clone git@github.com:epfl-lasa/kuka_fri.git
WORKDIR /tmp/kuka_fri
RUN if [ "${USE_SMID}" != "ON" ] ; \
    then wget https://gist.githubusercontent.com/matthias-mayr/0f947982474c1865aab825bd084e7a92/raw/244f1193bd30051ae625c8f29ed241855a59ee38/0001-Config-Disables-SIMD-march-native-by-default.patch \
    ; fi
RUN --mount=type=ssh  if [ "${USE_SMID}" != "ON" ] ; \
    then git am 0001-Config-Disables-SIMD-march-native-by-default.patch ; fi

# Transfer repo back to original user after root clone
WORKDIR /tmp
RUN chown -R ${USER}:${HOST_GID} kuka_fri

# Install kuka Fri as USER
USER ${USER}
RUN cd kuka_fri && ./waf configure && ./waf && sudo ./waf install

# Install SpaceVecAlg
RUN git clone --recursive https://github.com/costashatz/SpaceVecAlg.git
RUN cd SpaceVecAlg && mkdir build && cd build \
    && cmake -DCMAKE_BUILD_TYPE=Release -DENABLE_SIMD=ON -DPYTHON_BINDING=OFF .. \
    && make -j  ${NB_CPU_RESIZED}&& sudo make install

# Install RBDyn
RUN git clone --recursive https://github.com/costashatz/RBDyn.git
RUN cd RBDyn && mkdir build && cd build \
    && cmake -DCMAKE_BUILD_TYPE=Release -DENABLE_SIMD=ON -DPYTHON_BINDING=OFF .. \
    && make -j  ${NB_CPU_RESIZED}&& sudo make install

# Install mc_rbdyn_urdf
RUN git clone --recursive https://github.com/costashatz/mc_rbdyn_urdf.git
RUN cd mc_rbdyn_urdf && mkdir build && cd build \
    && cmake -DCMAKE_BUILD_TYPE=Release -DENABLE_SIMD=ON -DPYTHON_BINDING=OFF .. \
    && make -j  ${NB_CPU_RESIZED}&& sudo make install

# Install corrade
RUN git clone https://github.com/mosra/corrade.git
RUN cd corrade && git checkout 0d149ee9f26a6e35c30b1b44f281b272397842f5 \
    && mkdir build && cd build && cmake .. && make -j  ${NB_CPU_RESIZED}&& sudo make install

# Install robot_controller
RUN git clone https://github.com/epfl-lasa/robot_controllers.git
RUN cd robot_controllers && mkdir build && cd build \
    && cmake .. && make -j  ${NB_CPU_RESIZED}&& sudo make install

# Remove temp files
RUN sudo ldconfig
RUN rm -rf /tmp/*


FROM  iiwa-dependencies as iiwa-developpement
### Install IIWA ROS and toolkit
WORKDIR /home/${USER}/ros_ws/src

RUN git clone -b add_deep_camera https://github.com/RLoad/iiwa_ros.git
RUN git clone -b add_deep_camera https://github.com/RLoad/iiwa_toolkit.git

### Copy all src from github Robetarme_iwwa
COPY update_git.sh ./update_git.sh


FROM  iiwa-developpement as iiwa-simulation

### apt update and clean
RUN sudo apt update && sudo apt upgrade -y && sudo apt clean

WORKDIR /home/${USER}
RUN wget https://www.coppeliarobotics.com/files/CoppeliaSim_Edu_V4_6_0_rev6_Ubuntu20_04.tar.xz
RUN tar -xf CoppeliaSim_Edu_V4_6_0_rev6_Ubuntu20_04.tar.xz
RUN rm CoppeliaSim_Edu_V4_6_0_rev6_Ubuntu20_04.tar.xz

# Add zmq remote controle api plugin
RUN python3 -m pip install coppeliasim-zmqremoteapi-client
RUN pip install cbor pyzmq

# Add coppeliasim to bashrc but has to be sourced first to fill PYTHONPATH
# Use of && since a new terminal is instanciated for each RUN command
USER root
RUN source ~/.bashrc && echo PYTHONPATH=${PYTHONPATH}":/home/ros/CoppeliaSim_Edu_V4_6_0_rev6_Ubuntu20_04/programming/zmqRemoteApi/clients/python" >> ~/.bashrc
USER ${USER}

RUN sudo apt install -y \
    python3-vcstool \
    ros-noetic-rqt-joint-trajectory-controller \
    ros-noetic-moveit-commander \
    ros-noetic-teleop-twist-keyboard

WORKDIR /home/${USER}/ros_ws
RUN vcs import --input https://raw.githubusercontent.com/RobotnikAutomation/rbvogui_sim/noetic-devel/repos/rbvogui_sim.repos
RUN source /home/${USER}/.bashrc && rosdep install --from-paths src --ignore-src -y -r
RUN sudo dpkg -i src/rbvogui_common/libraries/*

WORKDIR /home/${USER}/ros_ws/src/rbvogui_sim/
RUN git checkout cart

WORKDIR /home/${USER}/ros_ws/src/rbvogui_common/
RUN git checkout cart

WORKDIR /home/${USER}/ros_ws/src/robotnik_sensors
RUN git checkout noetic-devel


FROM  iiwa-simulation as finalisation

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

RUN echo export ROS_MASTER_URI=http://localhost:11311 >> ~/.bashrc
RUN echo export ROS_IP=localhost >> ~/.bashrc
