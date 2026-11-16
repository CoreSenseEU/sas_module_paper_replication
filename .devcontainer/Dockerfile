ARG USER
ARG USER_UID=1000
ARG USER_GID=1000
ARG ROS_DISTRO=jazzy

FROM ros:jazzy-ros-core

# Redefine them to be used in this scope
ARG USER
ARG USER_UID
ARG USER_GID

# Delete user if it exists in container (e.g Ubuntu Noble: ubuntu)
RUN if id -u $USER_UID ; then userdel `id -un $USER_UID` ; fi

# Create the user
RUN groupadd --gid $USER_GID $USER \
    && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USER \
    && apt-get update && apt-get install -y \
        bash-completion \
        openssh-client \
        sudo \
    && rm -rf /var/lib/apt/lists/* \
    && rm /etc/apt/apt.conf.d/docker-clean \
    && echo $USER ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USER \
    && chmod 0440 /etc/sudoers.d/$USER \
    && chown -R $USER /root/ \
    && echo "source /opt/ros/jazzy/setup.bash" >> /home/${USER}/.bashrc


# Set the user as the default user
USER $USER
