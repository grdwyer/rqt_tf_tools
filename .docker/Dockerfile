FROM ros:foxy

# Install any needed packages then clean up apt cache
RUN apt-get update && apt-get install -y \
    ros-foxy-rqt \
    ros-foxy-rqt-common-plugins \
    ssh \
    python3-pip \
 && rm -rf /var/lib/apt/lists/*

RUN pip3 install PySide2

# Copies perfusion system stack into image
RUN mkdir -p /dev_ws/src/rqt_tf_tools
COPY . /dev_ws/src/rqt_tf_tools/

# Build the workspace so you're ready to go
WORKDIR /dev_ws
RUN ["/bin/bash", "-c", "source /opt/ros/foxy/setup.bash &&\
    rosdep install -i --from-path src --rosdistro foxy -y &&\
    colcon build"]


# RUN cp /dev_ws/src/perfusion_system/.docker/entrypoint.sh /
# RUN chmod 777 /entrypoint.sh
# ENTRYPOINT [ "/entrypoint.sh" ]

CMD ["bash"]
