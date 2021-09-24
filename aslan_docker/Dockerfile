FROM ros:melodic

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

# Change user
USER aslan_user

RUN cd && mkdir /home/$USERNAME/Aslan
COPY --chown=aslan_user ./src /home/$USERNAME/Aslan/src

# Install all dependencies with rosdep
RUN bash -c 'sudo apt-get update \
    && rosdep update \
    && cd ~/Aslan \
    && rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO \
    && sudo rm -rf /var/lib/apt/lists/*'

# Install build dependencies
RUN bash -c 'sudo apt-get update \
    && sudo apt-get install -y \
       python-catkin-tools \
    && sudo rm -rf /var/lib/apt/lists/*'

# Install Aslan
RUN bash -c 'source /opt/ros/melodic/setup.bash \
    && cd ~/Aslan \
    && catkin init \
    && catkin build'

# Install dev tools
RUN sudo apt-get update \
    && sudo apt-get install -y \
       gnome-terminal \
       dbus-x11 \
       vim \
       tmux \
       iproute2 \
       usbutils \
       gconf2 \
    && sudo rm -rf /var/lib/apt/lists/*

# Change Terminal Color
RUN gconftool-2 --set "/apps/gnome-terminal/profiles/Default/use_theme_background" --type bool false
RUN gconftool-2 --set "/apps/gnome-terminal/profiles/Default/use_theme_colors" --type bool false
RUN gconftool-2 --set "/apps/gnome-terminal/profiles/Default/background_color" --type string "#000000"

COPY run /home/$USERNAME/Aslan/run
WORKDIR /home/$USERNAME
