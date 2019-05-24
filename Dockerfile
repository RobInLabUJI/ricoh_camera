FROM osrf/ros:kinetic-desktop-full

ENV RICOH_WORKSPACE /ricoh_ws

RUN mkdir -p $RICOH_WORKSPACE/src
WORKDIR $RICOH_WORKSPACE/src

RUN git clone https://github.com/RobInLabUJI/ricoh_camera.git

RUN bash -c "source /opt/ros/kinetic/setup.bash && cd $RICOH_WORKSPACE && catkin_make"

COPY ros_entrypoint.sh /ros_entrypoint.sh
