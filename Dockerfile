##############################################################################
##                                 Base Image                               ##
##############################################################################
ARG ROS_DISTRO=noetic
# Ubuntu version depends on your provided ros distro
FROM osrf/ros:${ROS_DISTRO}-desktop
ENV TZ=Europe/Berlin
ENV TERM=xterm-256color
RUN ln -snf /usr/share/zoneinfo/${TZ} /etc/localtime && echo ${TZ} > /etc/timezone
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /etc/bash.bashrc

##############################################################################
##                                  User                                    ##
##############################################################################
ARG USER=docker
ARG PASSWORD=docker
ARG UID=1000
ARG GID=1000
ENV USER=${USER}
RUN groupadd -g ${GID} ${USER} && \
    useradd -m -u ${UID} -g ${GID} -p "$(openssl passwd -1 ${PASSWORD})" --shell $(which bash) ${USER} -G sudo
RUN echo "%sudo ALL=(ALL) NOPASSWD: ALL" > /etc/sudoers.d/sudogrp
RUN usermod -a -G video ${USER}

##############################################################################
##                                 Global Dependecies                       ##
##############################################################################
# Install default packages
RUN apt-get update && apt-get install --no-install-recommends -y \
    iputils-ping nano htop git sudo wget curl gedit python3-pip python3-catkin-tools gdb \
    && rm -rf /var/lib/apt/lists/*

# Install custom dependencies
# RUN apt-get update && apt-get install --no-install-recommends -y \
#     <YOUR_PACKAGE> \
#     && rm -rf /var/lib/apt/lists/*

RUN pip install -U \
    numpy scipy pybullet requests

##############################################################################
##                                 dependencies_ws                          ##
##############################################################################
USER ${USER}
RUN mkdir -p /home/${USER}/dependencies_ws/src
WORKDIR /home/${USER}/dependencies_ws/src

# ARG CACHE_BUST
# RUN git clone <linkToRepo> -b <branchName>


# Build dependencies_ws
WORKDIR /home/${USER}/dependencies_ws
RUN rosdep update --rosdistro ${ROS_DISTRO}
USER root
RUN apt-get update 
RUN rosdep install --from-paths src --ignore-src -r -y
RUN rm -rf /var/lib/apt/lists/*
USER ${USER}
RUN bash -c "source /opt/ros/$ROS_DISTRO/setup.bash && catkin build"
RUN echo "source /home/${USER}/dependencies_ws/devel/setup.bash" >> /home/$USER/.bashrc

##############################################################################
##                                 ros_ws                                   ##
##############################################################################
RUN mkdir -p /home/${USER}/ros_ws/src
WORKDIR /home/${USER}/ros_ws

# COPY <HOST_PATH> <CONTAINER_PATH>
COPY ./src ./src

# Build ros_ws
RUN bash -c "source /opt/ros/$ROS_DISTRO/setup.bash && catkin build"
RUN echo "source /home/${USER}/ros_ws/devel/setup.bash" >> /home/${USER}/.bashrc

# Build analog api pkg
USER root
WORKDIR /home/${USER}/ros_ws/src/analog_api
RUN pip install -e .

USER ${USER}
WORKDIR /home/${USER}/ros_ws

##############################################################################
##                           Start the container                            ##
##############################################################################

CMD ["bash"]