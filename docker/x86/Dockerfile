FROM ros:melodic-perception
WORKDIR /root

# Change default shell for RUN per https://stackoverflow.com/questions/20635472/using-the-run-instruction-in-a-dockerfile-with-source-does-not-work/39777387#39777387
SHELL ["/bin/bash", "-c"]

# Create and Initialize Catkin Workspace
RUN source /ros_entrypoint.sh && \
    mkdir -p wf_compare_ws/src && \
    cd ~/wf_compare_ws && \
    catkin_make

# Get Cartographer Source
RUN cd wf_compare_ws && \ 
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
RUN cd ~/wf_compare_ws/src/cartographer/scripts/ && \
    ./install_proto3.sh && \
    rm /etc/ros/rosdep/sources.list.d/20-default.list && \
    rosdep init && \
    rosdep update && \
    apt-get update && \
    rosdep install --from-paths ~/wf_compare_ws/src --ignore-src --rosdistro=${ROS_DISTRO} -y && \
    rm -rf /var/lib/apt/lists/*

# Build Cartographer
RUN source /ros_entrypoint.sh && \
    cd ~/wf_compare_ws && \
    catkin_make_isolated --install --use-ninja

# Modify avahi-config (this has to be done)
RUN perl -p -i -e 's|#allow-interfaces=eth0|allow-interfaces=wlp2s0,wlp2s1|g' /etc/avahi/avahi-daemon.conf

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
RUN echo 'alias avahi_go="/etc/init.d/dbus start && service avahi-daemon start"' >> ~/.bashrc && \
    echo 'alias test_cartographer_go="roslaunch cartographer_ros demo_backpack_2d.launch bag_filename:=/bags/cartographer_paper_deutsches_museum.bag"' >> ~/.bashrc

# Set GIT config based on local env vars
ARG GIT_NAME=
ARG GIT_EMAIL=
RUN git config --global user.email "$GIT_EMAIL" && \
    git config --global user.name "$GIT_NAME"

COPY ./wf_compare_entrypoint.sh /

ENTRYPOINT ["/wf_compare_entrypoint.sh"]
CMD ["bash"]
