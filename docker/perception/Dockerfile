# syntax=docker/dockerfile:1
# scout-perception: RealSense driver + YOLO detector (DeepStream nvinfer/NvSORT backend, ultralytics fallback).
# DeepStream 6.3 L4T image = Ubuntu 20.04, CUDA 11.4, cuDNN 8.6, TensorRT 8.5.2.2: the same as the Orin's
# JetPack 5.1.4, so the TensorRT engines and the custom YOLO parser built on the host load unchanged.
# Build context: ~/ros_utils  (docker compose -f docker/compose.yaml build perception)
FROM nvcr.io/nvidia/deepstream-l4t:6.3-samples

ARG DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# ROS Noetic (the image is Ubuntu 20.04 focal)
RUN apt-get update && apt-get install -y --no-install-recommends curl gnupg2 lsb-release ca-certificates \
 && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
 && echo "deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros/ubuntu focal main" \
      > /etc/apt/sources.list.d/ros1.list \
 && apt-get update && apt-get install -y --no-install-recommends \
      ros-noetic-ros-base ros-noetic-cv-bridge ros-noetic-image-transport ros-noetic-tf ros-noetic-tf2-ros \
      ros-noetic-ddynamic-reconfigure ros-noetic-diagnostic-updater ros-noetic-nodelet ros-noetic-visualization-msgs \
      python3-rosdep python3-catkin-tools python3-yaml python3-pip python3-dev python3-scipy \
      python3-gi python3-gst-1.0 gir1.2-gst-rtsp-server-1.0 python3-libnvinfer \
      git cmake build-essential libssl-dev libusb-1.0-0-dev libudev-dev pkg-config libgtk-3-dev \
      libjpeg-dev zlib1g-dev libpng-dev \
      cuda-nvcc-11-4 cuda-libraries-dev-11-4 \
 && rm -rf /var/lib/apt/lists/*
# the samples image has the CUDA runtime only; nvcc + headers (same 11.4.19 / cuBLAS 11.6.6.84 as the image's
# runtime) are needed to build librealsense (CUDA) and torchvision's CUDA ops
ENV CUDACXX=/usr/local/cuda-11.4/bin/nvcc CUDA_HOME=/usr/local/cuda-11.4

COPY docker/common/fetch_third_party.py /usr/local/bin/

# librealsense 2.57.5 from source, CUDA on, kernel (V4L) backend: as built on the host (~40-60 min, own layer)
COPY docker/pins/librealsense.* /opt/scout/pins/
RUN fetch_third_party.py /opt/scout/pins /opt/src librealsense \
 && cmake -S /opt/src/librealsense -B /tmp/rs-build -DCMAKE_BUILD_TYPE=Release -DBUILD_WITH_CUDA=ON \
      -DFORCE_RSUSB_BACKEND=OFF -DBUILD_EXAMPLES=OFF -DBUILD_GRAPHICAL_EXAMPLES=OFF -DBUILD_TOOLS=OFF \
      -DBUILD_UNIT_TESTS=OFF -DCMAKE_CUDA_ARCHITECTURES=87 \
 && make -C /tmp/rs-build -j"$(nproc)" install && ldconfig && rm -rf /tmp/rs-build /opt/src/librealsense

# torch's own shared-library deps (from ldd of the host venv's torch; cuDNN and libgomp are already in the image).
# Own layer, so the cached librealsense build above is kept.
RUN apt-get update && apt-get install -y --no-install-recommends libopenblas0-pthread libgfortran5 libnuma1 \
 && rm -rf /var/lib/apt/lists/*

# PyTorch for JetPack 5.1 (NVIDIA wheel, checksum = the one in the host venv) + torchvision 0.15.1 from source
ARG TORCH_WHL=torch-2.0.0a0+8aa34602.nv23.03-cp38-cp38-linux_aarch64.whl
ARG TORCH_SHA256=d7e5f1b23d443ce3ee306d2eedc8bb5773cca64f762925d098c1cbceca699643
RUN curl -sSLo /tmp/$TORCH_WHL "https://developer.download.nvidia.com/compute/redist/jp/v51/pytorch/${TORCH_WHL/+/%2B}" \
 && echo "$TORCH_SHA256  /tmp/$TORCH_WHL" | sha256sum -c - \
 && pip3 install --no-cache-dir --no-deps /tmp/$TORCH_WHL && rm /tmp/$TORCH_WHL
# pinned Python packages (torch's own deps included: filelock, networkx, sympy, typing-extensions); before
# torchvision, whose build imports torch
COPY docker/perception/requirements.txt /tmp/
RUN pip3 install --no-cache-dir --no-deps -r /tmp/requirements.txt
# torch 2.0's extension builder doesn't know sm_87 (Orin): build sm_86 + PTX, which runs on 8.7 and is what the
# host venv's torchvision contains (cuobjdump: _C.*.sm_86.cubin + .ptx)
ARG TORCHVISION_COMMIT=42759b1cc82bed60481c2802811595833e2ddd9b
RUN git clone -q https://github.com/pytorch/vision /tmp/vision && cd /tmp/vision && git checkout -q $TORCHVISION_COMMIT \
 && FORCE_CUDA=1 TORCH_CUDA_ARCH_LIST="8.6+PTX" BUILD_VERSION=0.15.1 MAX_JOBS="$(nproc)" pip3 install --no-cache-dir --no-deps --no-build-isolation . \
 && cd / && rm -rf /tmp/vision

# DeepStream Python bindings 1.1.8 (checksum = the one in the host venv)
ARG PYDS_SHA256=54f53b584fd668b62f86c6183c942e3e9ade5619465517864bb8180ae46bddae
RUN curl -sSLo /tmp/pyds-1.1.8-py3-none-linux_aarch64.whl \
      https://github.com/NVIDIA-AI-IOT/deepstream_python_apps/releases/download/v1.1.8/pyds-1.1.8-py3-none-linux_aarch64.whl \
 && echo "$PYDS_SHA256  /tmp/pyds-1.1.8-py3-none-linux_aarch64.whl" | sha256sum -c - \
 && pip3 install --no-cache-dir --no-deps /tmp/pyds-1.1.8-py3-none-linux_aarch64.whl && rm /tmp/pyds-*.whl

# import check without GPU. pyds and tensorrt are checked at run time instead: they link libnvbufsurface,
# libnvdla_compiler etc., Jetson host libraries
# that the NVIDIA container runtime mounts into a running container only
RUN python3 -c "import torch, torchvision, ultralytics"

# ROS workspace: vision_msgs + realsense-ros (pinned), rs_object_detection (incl. models: engines, custom parser)
WORKDIR /opt/scout/ws
# vision_opencv (cv_bridge) from source, as on the host: the image has NVIDIA's OpenCV 4.5.4 dev headers next to
# Ubuntu's 4.2 runtime that apt's cv_bridge links; realsense2_camera built against 4.5 headers + apt cv_bridge
# died with "undefined symbol: _ZN2cv3MatC1Ev" (cv::Mat::Mat())
COPY docker/pins/vision_msgs.* docker/pins/realsense-ros.* docker/pins/vision_opencv.* /opt/scout/pins/
RUN fetch_third_party.py /opt/scout/pins src vision_msgs realsense-ros vision_opencv
RUN rosdep init 2>/dev/null; rosdep update --rosdistro noetic && apt-get update \
 && rosdep install --from-paths src --ignore-src -r -y --rosdistro noetic --skip-keys "librealsense2" \
 && rm -rf /var/lib/apt/lists/*
RUN . /opt/ros/noetic/setup.sh && catkin_make -DCMAKE_BUILD_TYPE=Release -j"$(nproc)"
COPY detection_ws/src/rs_object_detection src/rs_object_detection
RUN . /opt/ros/noetic/setup.sh && catkin_make -DCMAKE_BUILD_TYPE=Release -j"$(nproc)" && rm -rf build

COPY docker/common/entrypoint.sh docker/common/healthcheck.sh /usr/local/bin/
COPY docker/launch/perception.launch /opt/scout/launch/
RUN printf 'source /opt/ros/noetic/setup.bash\nsource /opt/scout/ws/devel/setup.bash\n' > /opt/scout/setup.bash \
 && useradd -u 1000 -m -d /home/agile -G video agile \
 && install -d -o agile -g agile /home/agile/.nv   # CUDA JIT cache: a named volume (compose) keeps it
ENV SCOUT_SETUP=/opt/scout/setup.bash SCOUT_CONTAINER=1 PYTHONUNBUFFERED=1
USER agile
WORKDIR /home/agile
ENTRYPOINT ["entrypoint.sh"]
CMD ["roslaunch", "--wait", "/opt/scout/launch/perception.launch"]
