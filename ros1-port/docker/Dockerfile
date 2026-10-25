# Python 3.10 is Ubuntu 22.04's native ABI. No host ROS/Python is mounted.
FROM nvidia/cuda:12.6.3-cudnn-runtime-ubuntu22.04@sha256:46cb48a4abfbc40c836fe57bc05a07101b6458fffc63bbdfd6a50db98c9358bd AS tools
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ENV DEBIAN_FRONTEND=noninteractive ROS_DISTRO=noetic ROS_VERSION=1 ROS_PYTHON_VERSION=3 \
    PYTHONDONTWRITEBYTECODE=1 PIP_DISABLE_PIP_VERSION_CHECK=1
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake git curl ca-certificates python3-dev python3-pip \
    python3-setuptools python3-yaml python3-pyqt5 python3-sip-dev \
    libboost-all-dev libgl1-mesa-dev libglu1-mesa-dev libx11-dev \
    iproute2 procps file && rm -rf /var/lib/apt/lists/*
COPY requirements.txt python310.lock.txt /opt/rmt/
RUN python3 -m pip install -r /opt/rmt/python310.lock.txt && python3 -m pip check
RUN rosdep init && rosdep update --include-eol-distros --rosdistro noetic
WORKDIR /opt/noetic-src
COPY noetic.rosinstall /opt/rmt/noetic.rosinstall

FROM tools AS runtime
COPY fetch_noetic.py noetic.sha256.json /opt/rmt/
RUN python3 /opt/rmt/fetch_noetic.py /opt/rmt/noetic.rosinstall \
    /opt/rmt/noetic.sha256.json src
COPY install_rosdeps.py /opt/rmt/install_rosdeps.py
RUN apt-get update && python3 /opt/rmt/install_rosdeps.py && rm -rf /var/lib/apt/lists/*
COPY rosconsole-log4cxx.patch /opt/rmt/rosconsole-log4cxx.patch
RUN python3 -c "import hashlib; assert hashlib.sha256(open('/opt/rmt/rosconsole-log4cxx.patch','rb').read()).hexdigest() == 'cfdd9c6c1ca22d0ddd80a3075cd8a1cde2cc6edf45b30e4042fb99f23e21a681'" \
    && patch --directory src/rosconsole --strip=1 --fuzz=0 < /opt/rmt/rosconsole-log4cxx.patch
RUN --mount=type=cache,id=rmt-noetic-build,target=/opt/noetic-src/build_isolated \
    --mount=type=cache,id=rmt-noetic-devel,target=/opt/noetic-src/devel_isolated \
    ./src/catkin/bin/catkin_make_isolated --install --install-space /opt/ros/noetic \
    -DPYTHON_EXECUTABLE=/usr/bin/python3 -DPYTHON_INCLUDE_DIR=/usr/include/python3.10 \
    -DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.10.so \
    -DCMAKE_CXX_STANDARD=17 -DCMAKE_BUILD_TYPE=Release -j4
RUN --mount=type=cache,id=rmt-noetic-build,target=/opt/noetic-src/build_isolated \
    --mount=type=cache,id=rmt-noetic-devel,target=/opt/noetic-src/devel_isolated \
    source /opt/ros/noetic/setup.bash \
    && cmake --build build_isolated/rosconsole --target run_tests -j4 \
    && catkin_test_results build_isolated/rosconsole/test_results \
    && cp -a build_isolated/rosconsole/test_results /opt/rmt/rosconsole-test-results
RUN source /opt/ros/noetic/setup.bash && python3 -c \
    'import sys, tf2_py, tf2_ros, rospy; assert sys.version_info[:2] == (3,10); print(tf2_py.__file__)' \
    && python3 -m pip check && python3 -m pip freeze > /opt/rmt/pip-freeze.txt \
    && dpkg-query -W > /opt/rmt/dpkg-manifest.txt
COPY entrypoint.sh /opt/rmt/entrypoint.sh
ENTRYPOINT ["/bin/bash", "/opt/rmt/entrypoint.sh"]
CMD ["bash"]
