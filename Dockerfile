from ros:kinetic

ENV PYTHONIOENCODING=UTF-8 \
    CATKIN_WS=/catkin_ws

RUN apt-get update && apt-get install -y \
    python-pip \
 && rm -rf /var/lib/apt/lists/*

RUN pip install catkin_tools wstool

COPY ./ /$CATKIN_WS/src/godel
WORKDIR /$CATKIN_WS

RUN wstool init --shallow src src/godel/godel.repos

RUN bash -c "source /opt/ros/$ROS_DISTRO/setup.bash && apt update && rosdep install -y --ignore-packages-from-source --from-paths $CATKIN_WS/src"

RUN bash -c "source /opt/ros/$ROS_DISTRO/setup.bash && catkin build --no-status"