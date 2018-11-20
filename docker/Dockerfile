FROM ros:kinetic

RUN apt-get update && apt-get install -y software-properties-common && \
    add-apt-repository -y ppa:timn/clips && \
    apt-get update && apt-get upgrade -y && \
    apt-get install -y libmodbus-dev libclips-dev clips libclipsmm-dev \
            protobuf-compiler libprotobuf-dev libprotoc-dev \
            libmodbus-dev \
            libglibmm-2.4-dev libgtkmm-3.0-dev libncurses5-dev \
            libncursesw5-dev libyaml-cpp-dev libavahi-client-dev git \
            libssl-dev libelf-dev mongodb-clients \
            mongodb libzmq3-dev \
            scons libboost-all-dev \
            ros-kinetic-roslint

WORKDIR /root/catkin_ws/src/

RUN git clone -b erl-2018 https://github.com/industrial-robotics/atwork_refbox_comm && \
    cd atwork_refbox_comm && \
    git submodule init && \
    git submodule update && \
    cd atwork_refbox && \
    git checkout erl-2018

WORKDIR /root/catkin_ws/src/
RUN git clone -b erl-2018 https://github.com/industrial-robotics/atwork_refbox_ros_client

WORKDIR /root/catkin_ws
RUN [ "bash", "-c", ". /opt/ros/kinetic/setup.bash && rosdep update && rosdep install --from-paths src --ignore-src -r -y && catkin_make" ]

CMD [ "bash", "-c", ". /opt/ros/kinetic/setup.bash && . devel/setup.bash && roslaunch atwork_refbox_ros_client robot_example_ros.launch" ]
