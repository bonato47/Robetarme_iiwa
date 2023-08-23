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

### Add the user to the current GID of the host to avoid permisson issues in volumes
# AICA uses the same name for user and user group
ENV USER_GROUP=${USER}
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
RUN sudo apt-get update && sudo apt-get install -y \
    bash-completion \
    silversearcher-ag \
    apt-transport-https \
    less \
    alsa-utils \
    ros-${ROS_DISTRO}-ros-control \
    ros-${ROS_DISTRO}-ros-controllers \
    && sudo apt-get upgrade -y && sudo apt-get clean
    
    
FROM ros-ws as inverse-kinematics


# Install qpPOASES
RUN git clone https://github.com/coin-or/qpOASES.git
RUN  cd qpOASES&& mkdir build && cd build \
    && cmake .. && make -j && sudo make install


# Install trak_ik
WORKDIR /home/${USER}
RUN git clone https://bitbucket.org/traclabs/trac_ik.git
RUN cp -R trac_ik ros_ws/src/
RUN rm trac_ik -r

# Install pinochio and control libraire from epfl-lasa#
WORKDIR /home/${USER}
RUN git clone https://github.com/epfl-lasa/control-libraries.git --branch v6.3.1  --single-branch
WORKDIR /home/${USER}/control-libraries/source
RUN sudo bash install.sh -y

#isntall universal robot
WORKDIR /home/${USER}
RUN git clone https://github.com/ros-industrial/universal_robot.git
RUN cp -R universal_robot ros_ws/src/
RUN rm universal_robot -r

# Install relaxed ik
RUN curl -sSL https://bootstrap.pypa.io/get-pip.py -o get-pip.py
RUN sudo apt-get install -y ros-noetic-kdl-parser ros-noetic-kdl-parser-py
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
RUN sudo apt-get install -y build-essential cmake
RUN sudo chmod 777 ~/.bashrc
RUN curl --proto '=https' --tlsv1.3 https://sh.rustup.rs -sSf | sh -s -- -y
RUN sudo chmod 644 ~/.bashrc
ENV PATH="/home/${USER}/.cargo/bin:${PATH}"

# Need to be root to use ssh inside docker build
WORKDIR /home/${USER}
USER root
RUN --mount=type=ssh git clone git@github.com:lmunier/ncollide.git --branch fix_relaxed_ik

# Transfer repo back to original user after root clone
RUN chown -R ${USER}:${HOST_GID} ncollide
USER ${USER}

WORKDIR /home/${USER}/ros_ws/src/relaxed_ik_ros1/relaxed_ik_core

# Modify crates used to have the ones with the fixes
RUN sed -i 's/ncollide2d = \"0.19\"/ncollide2d = \{ path = \"..\/..\/..\/..\/ncollide\/build\/ncollide2d\" \}/' Cargo.toml
RUN sed -i 's/ncollide3d = \"0.21.0\"/ncollide3d = \{ path = \"..\/..\/..\/..\/ncollide\/build\/ncollide3d\" \}/' Cargo.toml
RUN cargo build
RUN cargo fix --lib -p relaxed_ik_core --allow-dirty
   
FROM inverse-kinematics as iiwa-dependencies

# Install gazebo (9 or 11 depending on distro)
WORKDIR /home/${USER}
RUN sudo apt-get update
RUN if [ "$ROS_DISTRO" = "noetic" ] ; then sudo apt-get install -y gazebo11 ; fi
RUN if [ "$ROS_DISTRO" = "melodic" ] ; then sudo apt-get install -y gazebo9 ; fi

# Install gazebo ros packages
RUN sudo apt install -y ros-${ROS_DISTRO}-gazebo-ros-pkgs ros-${ROS_DISTRO}-gazebo-ros-control

# Install trak_ik_lib
RUN sudo apt install -y ros-${ROS_DISTRO}-trac-ik

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

# Remove git identity
RUN git config --global --unset user.name
RUN git config --global --unset user.email

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
	&& make -j && sudo make install

# Install RBDyn
RUN git clone --recursive https://github.com/costashatz/RBDyn.git
RUN cd RBDyn && mkdir build && cd build \
    && cmake -DCMAKE_BUILD_TYPE=Release -DENABLE_SIMD=ON -DPYTHON_BINDING=OFF .. \
    && make -j && sudo make install

# Install mc_rbdyn_urdf
RUN git clone --recursive https://github.com/costashatz/mc_rbdyn_urdf.git
RUN cd mc_rbdyn_urdf && mkdir build && cd build \
    && cmake -DCMAKE_BUILD_TYPE=Release -DENABLE_SIMD=ON -DPYTHON_BINDING=OFF .. \
    && make -j && sudo make install

# Install corrade
RUN git clone https://github.com/mosra/corrade.git
RUN cd corrade && git checkout 0d149ee9f26a6e35c30b1b44f281b272397842f5 \
    && mkdir build && cd build && cmake .. && make -j && sudo make install

# Install robot_controller
RUN git clone https://github.com/epfl-lasa/robot_controllers.git
RUN cd robot_controllers && mkdir build && cd build \
    && cmake .. && make -j && sudo make install

# Install qpPOASES
RUN git clone https://github.com/coin-or/qpOASES.git
RUN  cd qpOASES&& mkdir build && cd build \
    && cmake .. && make -j && sudo make install

# Remove temporari files
RUN sudo ldconfig
RUN rm -rf /tmp/*

### Install IIWA ROS
WORKDIR /home/${USER}/ros_ws/src
RUN git clone https://github.com/epfl-lasa/iiwa_ros.git

### Install IIWA toolkit
RUN git clone https://github.com/fkhadivar/iiwa_toolkit.git


FROM  iiwa-dependencies as finalisation

### Copy all src from github Robetarme_iwwa
COPY update_git.sh ./update_git.sh

# Give bashrc back to user
WORKDIR /home/${USER}
RUN sudo chown -R ${USER}:${HOST_GID} .bashrc

# Add cmake option to bash rc if needed
RUN if [ "${USE_SIMD}" = "ON" ] ; \
    then echo "export ENABLE_SIMD=ON" >> /home/${USER}/.bashrc ; fi

### Build ros workspace
WORKDIR /home/${USER}/ros_ws
RUN source /home/${USER}/.bashrc && rosdep install --from-paths src --ignore-src -r -y
RUN source /home/${USER}/.bashrc && catkin_make;

### Final apt clean
RUN sudo apt update && sudo apt upgrade -y && sudo apt clean
