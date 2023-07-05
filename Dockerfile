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

# Setup git identity
RUN git config --global user.name "${GIT_NAME}"
RUN git config --global user.email "${GIT_EMAIL}"

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
   

FROM ros-ws as iiwa-dependencies

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
RUN git clone --recursive https://github.com/jrl-umi3218/SpaceVecAlg.git
RUN cd SpaceVecAlg && mkdir build && cd build \
    && cmake -DCMAKE_BUILD_TYPE=Release -DPYTHON_BINDING=OFF .. \
    && make -j && sudo make install

# Install RBDyn
RUN git clone --recursive https://github.com/jrl-umi3218/RBDyn.git
RUN cd RBDyn && mkdir build && cd build \
    && cmake -DCMAKE_BUILD_TYPE=Release -DPYTHON_BINDING=OFF .. \
    && make -j && sudo make install

# Install mc_rbdyn_urdf
RUN git clone --recursive -b v1.1.0 https://github.com/jrl-umi3218/mc_rbdyn_urdf.git
RUN cd mc_rbdyn_urdf && mkdir build && cd build \
    && cmake -DCMAKE_BUILD_TYPE=Release -DPYTHON_BINDING=OFF .. \
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

FROM iiwa-dependencies as inverse-kinematics

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


#install eigen
#WORKDIR /home/${USER}
#RUN wget -c https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.tar.gz -O - | tar -xz
#RUN cd eigen-3.4.0 && mkdir build && cd build && cmake .. && make install
#RUN cd ../.. && rm -r eigen-3.4.0

#WORKDIR /home/${USER}/control-libraries/source
#RUN mkdir build && cd build
#RUN cmake -DCMAKE_BUILD_TYPE=Release ..
#RUN make -j
#RUN make install

#WORKDIR /home/${USER}/control-libraries/protocol
#RUN sudo bash install.sh --auto

#COPY --from=ghcr.io/epfl-lasa/control-libraries/development-dependencies:latest /usr/local/include/google /usr/local/include/google
#COPY --from=ghcr.io/epfl-lasa/control-libraries/development-dependencies:latest /usr/local/lib/libproto* /usr/local/lib
#COPY --from=ghcr.io/epfl-lasa/control-libraries/development-dependencies:latest /usr/local/bin/protoc /usr/local/bin
#RUN sudo ldconfig

#RUN echo "deb [arch=amd64] http://robotpkg.openrobots.org/packages/debian/pub $(lsb_release -cs) robotpkg" | sudo tee /etc/apt/sources.list.d/robotpkg.list
#RUN  curl http://robotpkg.openrobots.org/packages/debian/robotpkg.key | sudo apt-key add -
#RUN  sudo apt update
#RUN  sudo apt install -qqy robotpkg-py3*-pinocchio
#RUN export PATH=/opt/openrobots/bin:$PATH
#RUN export PKG_CONFIG_PATH=/opt/openrobots/lib/pkgconfig:$PKG_CONFIG_PATH
#RUN export LD_LIBRARY_PATH=/opt/openrobots/lib:$LD_LIBRARY_PATH
#RUN export PYTHONPATH=/opt/openrobots/lib/python3.8.10/site-packages:$PYTHONPATH 
#RUN export CMAKE_PREFIX_PATH=/opt/openrobots:$CMAKE_PREFIX_PATH


FROM inverse-kinematics as finalisation

### Add environement variables to bashrc
WORKDIR /home/${USER}


### Copy all src from github Robetarme_iwwa
COPY --chown=${USER} ./src ./ros_ws/src

# Give bashrc back to user
RUN sudo chown -R ${USER}:${HOST_GID} .bashrc

# Add cmake option to bash rc if needed
RUN if [ "${USE_SIMD}" = "ON" ] ; \
    then echo "export ENABLE_SIMD=ON" >> /home/${USER}/.bashrc ; fi

### Build ros workspace
WORKDIR /home/${USER}/ros_ws
RUN source /home/${USER}/.bashrc && rosdep install --from-paths src --ignore-src -r -y
RUN source /home/${USER}/.bashrc && catkin_make;

### Final apt clean
RUN sudo apt update && sudo apt upgrade -y   && sudo apt clean
