# ROS 2 Jazzy image that builds both packages (Python nodes, C++ controller) and runs every test.
#   docker build -t ugv-patrol .
#   docker run --rm ugv-patrol                 # all tests (scripts/ci.sh)
#   docker run --rm -it ugv-patrol bash        # shell with the workspace sourced
FROM ros:jazzy-ros-base

RUN apt-get update && apt-get install -y --no-install-recommends \
        python3-numpy python3-opencv python3-yaml python3-pytest \
        ros-jazzy-nav-msgs ros-jazzy-visualization-msgs ros-jazzy-tf2-ros \
        cmake g++ libgtest-dev libyaml-cpp-dev \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ws
COPY . src/ugv_patrol_planner
# cpp/ is its own ROS 2 package (C++ controller); colcon does not look inside another package.
RUN ln -s /ws/src/ugv_patrol_planner/cpp /ws/src/ugv_patrol_planner_cpp \
    && . /opt/ros/jazzy/setup.sh && colcon build --symlink-install --event-handlers console_direct- \
       --cmake-args -DCMAKE_BUILD_TYPE=Release \
    && echo 'source /opt/ros/jazzy/setup.bash && source /ws/install/setup.bash' >> /root/.bashrc

WORKDIR /ws/src/ugv_patrol_planner
SHELL ["/bin/bash", "-c"]
CMD source /opt/ros/jazzy/setup.bash && source /ws/install/setup.bash && bash scripts/ci.sh /ws/build_cpp
