# Use a base image
FROM ros:humble-ros-base-jammy

# Install necessary packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    # tooling packages
    clangd \
    python3-pip \
    # ROS packages
    ros-humble-hardware-interface \
    ros-humble-generate-parameter-library \
    ros-humble-controller-interface \
    ros-humble-moveit \
    # ROS realsense
    ros-humble-realsense2-* \
    && rm -rf /var/lib/apt/lists/*

RUN pip install --no-cache-dir clangd-tidy

# Install cpp-member-accessor, used to test private methods in unit tests.
RUN mkdir -p ~/sources && \
    cd ~/sources && \
    git clone https://github.com/hliberacki/cpp-member-accessor.git && \
    cd cpp-member-accessor && \
    # This was the last commit (on Jan 12, 2021), the lastest available at the time.
    git checkout e72109f1400a9b70cdaaae1bf0e9192900dcc332 && \
    mkdir build && \
    cd build && \
    cmake .. && \
    make && \
    sudo make install
