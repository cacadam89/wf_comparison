FROM ros:melodic-perception
WORKDIR /root

# Change default shell for RUN per https://stackoverflow.com/questions/20635472/using-the-run-instruction-in-a-dockerfile-with-source-does-not-work/39777387#39777387
SHELL ["/bin/bash", "-c"]

# Create and Initialize Catkin Workspace
RUN source /ros_entrypoint.sh && \
    mkdir -p wf_comparison_ws/src && \
    cd ~/wf_comparison_ws && \
    catkin_make

# Get Cartographer Source
RUN cd wf_comparison_ws && \ 
    wstool init src && \
    wstool merge -t src https://raw.githubusercontent.com/googlecartographer/cartographer_ros/master/cartographer_ros.rosinstall && \
    wstool update -t src

# Install some dependices for Cartographer
RUN apt-get update &&  apt-get install -y \
        apt-utils \
        ninja-build \
        python-rosdep \
        python-sphinx \
        python-wstool \
    && rm -rf /var/lib/apt/lists/*

# Intall Proto3 (for Cartographer) and Remainding Dependencies
RUN cd ~/wf_comparison_ws/src/cartographer/scripts/ && \
    ./install_proto3.sh && \
    rm /etc/ros/rosdep/sources.list.d/20-default.list && \
    rosdep init && \
    rosdep update && \
    apt-get update && \
    rosdep install --from-paths ~/wf_comparison_ws/src --ignore-src --rosdistro=${ROS_DISTRO} -y && \
    rm -rf /var/lib/apt/lists/*

# Build Cartographer
RUN source /ros_entrypoint.sh && \
    cd ~/wf_comparison_ws && \
    catkin_make_isolated --install --use-ninja

# # Clone and Build WF_COMPARISON
# RUN source /ros_entrypoint.sh && \
#     cd wf_comparison_ws/src && \
#     git clone --recurse-submodules https://github.com/cacadam89/wf_comparison.git wf_comparison
RUN source /ros_entrypoint.sh && \
    cd wf_comparison_ws/src && \
    git clone --recurse-submodules https://github.com/cacadam89/wf_comparison.git wf_comparison  && \
    cd .. && \
    catkin_make_isolated

# Modify avahi-config (this has to be done)
RUN perl -p -i -e 's|#allow-interfaces=eth0|allow-interfaces=wlp2s0,wlp2s1|g' /etc/avahi/avahi-daemon.conf

# Set Env Variables for NVIDIA (or else rviz wont work) - https://wiki.ros.org/docker/Tutorials/Hardware%20Acceleration
ENV NVIDIA_VISIBLE_DEVICES ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

# ---------------------------- DEBUG -start ------------------------------ #
# Install 'ping'
RUN apt-get update && apt-get install -y \
        iputils-ping \
    && rm -rf /var/lib/apt/lists/*
    
# Install 'locate'
RUN apt-get update && apt-get install -y \
        mlocate \
    && rm -rf /var/lib/apt/lists/* &&\
    updatedb

# Install 'xclock'
RUN apt-get update && apt-get install -y \
        x11-apps \ 
    && rm -rf /var/lib/apt/lists/*
# ---------------------------- DEBUG - end ------------------------------ #

# ---------------------------- Reset argument ------------------------------ #
ARG rebuild=0
# -------------------------------------------------------------------------- #

# Add aliases to bashrc in docker containers
RUN echo 'alias test_cartographer_go="roslaunch cartographer_ros demo_backpack_2d.launch bag_filename:=/bags/cartographer_paper_deutsches_museum.bag"' >> ~/.bashrc

# # Re-pull & Build if ARG rebuild changed
RUN source /ros_entrypoint.sh && \
    cd wf_comparison_ws/src/wf_comparison && \
    git pull && \
    catkin_make_isolated

# Set GIT config based on local env vars
ARG GIT_NAME=
ARG GIT_EMAIL=
RUN git config --global user.email "$GIT_EMAIL" && \
    git config --global user.name "$GIT_NAME"

COPY ./wf_comparison_entrypoint.sh /

ENTRYPOINT ["/wf_comparison_entrypoint.sh"]
CMD ["bash"]

# my BUILD and RUN commands:
# docker build $WF_COMPARE_PATH/docker/x86/ -t wf_comparison --build-arg GIT_NAME=$DOCKER_GIT_NAME --build-arg GIT_EMAIL=$DOCKER_GIT_EMAIL
# docker run -it --gpus all --privileged --env-file $WF_COMPARE_PATH/docker/.env -v $WF_COMPARE_PATH/data:/bags:ro -env="DISPLAY" --env="QT_X11_NO_MITSHM=1" --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" -env="XAUTHORITY=$XAUTH" --volume="$XAUTH:$XAUTH" --runtime=nvidia --network host wf_comparison:latest bash