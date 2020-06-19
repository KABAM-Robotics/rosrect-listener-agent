FROM osrf/ros:melodic-desktop-full

RUN apt-get update && \
    apt-get install -y --no-install-recommends screen \
    libcpprest-dev

RUN apt-get update && \
    apt-get install -y  && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

WORKDIR /home/catkin_diag_agent
COPY . src/rosrect-listener-agent

# ARG RUN_TESTS
RUN /ros_entrypoint.sh catkin_make && sed -i '$isource "/home/catkin_diag_agent/devel/setup.bash"' /ros_entrypoint.sh
# RUN /bin/bash -c "source /opt/ros/melodic/setup.bash;"
# RUN /bin/bash -c "source /home/catkin_diag_agent/devel/setup.bash;"
ENTRYPOINT ["/ros_entrypoint.sh"]