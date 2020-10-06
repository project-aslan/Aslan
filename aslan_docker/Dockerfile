FROM nvidia/opengl:1.0-glvnd-devel-ubuntu18.04

RUN apt-get update && apt-get install -y --no-install-recommends \
        pkg-config \
        libxau-dev \
        libxdmcp-dev \
        libxcb1-dev \
        libxext-dev \
        libx11-dev

ARG DEBIAN_FRONTEND=noninteractive


# COPY --from=nvidia/opengl:1.0-glvnd-runtime-ubuntu18.04 \
#   /usr/local/lib/x86_64-linux-gnu \
#   /usr/local/lib/x86_64-linux-gnu

COPY --from=nvidia/opengl:1.0-glvnd-runtime-ubuntu18.04 \
   /usr/share/glvnd/egl_vendor.d/10_nvidia.json \
   /usr/share/glvnd/egl_vendor.d/10_nvidia.json

# RUN echo '/usr/share/glvnd/egl_vendor.d/' >> /etc/ld.so.conf.d/glvnd.conf && \
#     ldconfig && \
#     echo '/usr/local/$LIB/libGL.so.1' >> /etc/ld.so.preload && \
#     echo '/usr/local/$LIB/libEGL.so.1' >> /etc/ld.so.preload

RUN apt-get update && apt-get install -y dialog apt-utils && apt-get install -y curl

RUN apt-get update && apt-get install -y --no-install-recommends \
    	libglvnd0 libglvnd0:amd64 \
    	libgl1 libgl1:amd64 \
 	    libglx0 libglx0:amd64 \
 	    libegl1 libegl1:amd64 \
 	    libgles2 libgles2:amd64 && \
         rm -rf /var/lib/apt/lists/*

# COPY 10_nvidia.json /usr/share/glvnd/egl_vendor.d/10_nvidia.json


# nvidia-container-runtime
ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics


RUN apt-get update && apt-get install -y \
        software-properties-common \
        wget curl git cmake cmake-curses-gui \
        libboost-all-dev \
        libflann-dev \
        libgsl0-dev \
        libgoogle-perftools-dev \
        libeigen3-dev

# Intall some basic GUI and sound libs
RUN apt-get update && apt-get install -y \
        xz-utils file locales dbus-x11 pulseaudio dmz-cursor-theme \
        fonts-dejavu fonts-liberation hicolor-icon-theme \
        libcanberra-gtk3-0 libcanberra-gtk-module libcanberra-gtk3-module \
        libasound2 libgtk2.0-0 libdbus-glib-1-2 libxt6 libexif12 \
        libgl1-mesa-glx libgl1-mesa-dri language-pack-en \
        && update-locale LANG=en_US.UTF-8 LC_MESSAGES=POSIX

# Intall some basic GUI tools
RUN apt-get update && apt-get install -y \
        cmake-qt-gui \
        gnome-terminal

# Intall ROS
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
RUN apt-get update && apt-get install -y --allow-unauthenticated ros-melodic-desktop-full
RUN apt-get update && apt-get install -y --allow-unauthenticated python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential python-catkin-tools
# RUN apt-get install -y --allow-unauthenticated ros-melodic-can-msgs ros-melodic-socketcan-interface ros-melodic-catkin python-catkin-tools
# RUN apt-get update && apt-get install -y --allow-unauthenticated ros-melodic-controller-manager ros-melodic-ros-control ros-melodic-ros-controllers ros-melodic-gazebo-ros-control ros-melodic-joystick-drivers ros-melodic-nmea-msgs ros-melodic-nmea-navsat-driver ros-melodic-sound-play ros-melodic-jsk-visualization ros-melodic-grid-map ros-melodic-gps-common
RUN apt-get update && apt-get install -y --allow-unauthenticated libnlopt-dev freeglut3-dev qtbase5-dev libqt5opengl5-dev libssh2-1-dev libarmadillo-dev libpcap-dev libgl1-mesa-dev libglew-dev python-wxgtk3.0 software-properties-common libmosquitto-dev libyaml-cpp-dev python-flask python-requests

# RUN DEBIAN_FRONTEND=noninteractive
#RUN apt-get install -y --allow-unauthenticated lxde-core lxterminal tightvncserver
#RUN rm -rf /var/lib/apt/lists/*

# Add basic user
ENV USERNAME aslan_user
ENV PULSE_SERVER /run/pulse/native
RUN useradd -m $USERNAME && \
        echo "$USERNAME:$USERNAME" | chpasswd && \
        usermod --shell /bin/bash $USERNAME && \
        usermod -aG sudo $USERNAME && \
        echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers.d/$USERNAME && \
        chmod 0440 /etc/sudoers.d/$USERNAME && \
        # Replace 1000 with your user/group id
        usermod  --uid 1000 $USERNAME && \
        groupmod --gid 1000 $USERNAME

# Setup .bashrc for ROS
RUN echo "source /opt/ros/melodic/setup.bash" >> /home/$USERNAME/.bashrc && \
        #Fix for qt and X server errors
        echo "export QT_X11_NO_MITSHM=1" >> /home/$USERNAME/.bashrc && \
        #Set ROS hostname to localhost
#        echo "export ROS_HOSTNAME=localhost" >> /home/$USERNAME/.bashrc && \
        # cd to home on login
        echo "cd" >> /home/$USERNAME/.bashrc

# Change user
USER aslan_user

RUN sudo rosdep init \
        && rosdep update \
        && echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc

# Install Aslan
RUN cd && mkdir /home/$USERNAME/Aslan
COPY --chown=aslan_user ./ /home/$USERNAME/Aslan/
RUN /bin/bash -c 'source /opt/ros/melodic/setup.bash; cd /home/$USERNAME/Aslan; rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO; catkin init; catkin build'
RUN echo "source /home/$USERNAME/Aslan/devel/setup.bash" >> /home/$USERNAME/.bashrc

# Setting
ENV LANG="en_US.UTF-8"
RUN echo "export LANG=\"en_US.UTF-8\"" >> /home/$USERNAME/.bashrc

# Install dev tools
RUN sudo apt-get -y --allow-unauthenticated install vim tmux iproute2 usbutils
RUN sudo apt-get -y --allow-unauthenticated update && sudo apt-get install -y --allow-unauthenticated gconf2
# # Change Terminal Color
RUN gconftool-2 --set "/apps/gnome-terminal/profiles/Default/use_theme_background" --type bool false
RUN gconftool-2 --set "/apps/gnome-terminal/profiles/Default/use_theme_colors" --type bool false
RUN gconftool-2 --set "/apps/gnome-terminal/profiles/Default/background_color" --type string "#000000"

LABEL com.nvidia.volumes.needed="nvidia_driver"
ENV PATH /usr/local/nvidia/bin:${PATH}
ENV LD_LIBRARY_PATH /usr/local/nvidia/lib:/usr/local/nvidia/lib64:${LD_LIBRARY_PATH}

# Default CMD
CMD ["/bin/bash"]