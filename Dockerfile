FROM ros:melodic-ros-core

RUN apt-get update && \
    apt-get install -y --no-install-recommends screen \
    libcpprest-dev \
    g++ \
    make

RUN apt-get update && \
    apt-get install -y  && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

WORKDIR /home/catkin_diag_agent
COPY . src/rosrect-listener-agent

RUN /ros_entrypoint.sh catkin_make && sed -i '$isource "/home/catkin_diag_agent/devel/setup.bash"' /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]