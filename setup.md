
# Jetson Installation Instructions

## Jetpack & System Setup

* Install Jetpack 5.1
* Install venv
* Create venv

### Install System Dependencies

```bash
sudo apt install -y \
    libopenblas-base libopenblas-dev liblapack-dev libblas-dev \
    libjpeg-dev zlib1g-dev libpng-dev
sudo apt install cmake libopenblas-dev libhdf5-dev libjpeg-dev libpng-dev gifsicle
sudo apt install python3-dev python3.8-dev

```

---

## Python Environment Setup

**Note:** Install torch in venv from nvidia repository (2.0.0)

### Basic Dependencies

```bash
pip install "numpy==1.24"
pip install scipy pandas matplotlib
pip install scikit-learn scikit-image scikit-fmm
pip install pytest click natsort yacs trimesh
pip install h5py imageio pynput
pip install pybind11
pip install onnx
pip install "huggingface_hub<0.24"
# pip install torchvision==0.15.1 (This matches our torch 2.0.0) #installs non-cuda torchvision, need to build from source

```

### Build Torchvision from Source

```bash
export TORCH_CUDA_ARCH_LIST="8.6"
git clone [https://github.com/pytorch/vision.git](https://github.com/pytorch/vision.git)
cd vision
git checkout v0.15.1
export BUILD_VERSION=0.15.1
python setup.py install

```

### Additional ML Libraries

```bash
# continue installs
pip install timm
pip install ftfy regex tqdm
pip install git+[https://github.com/openai/CLIP.git](https://github.com/openai/CLIP.git) --no-deps
# verify clip with:
# python -c "import clip; print(clip.available_models())"
# output:  # ['RN50', 'RN101', 'RN50x4', 'RN50x16', 'RN50x64', 'ViT-B/32', 'ViT-B/16', 'ViT-# L/14', 'ViT-L/14@336px']

pip install wheel

```

### Packages Requiring Build

```bash
pip install numpy-quaternion
pip install sophuspy
pip install pybullet
pip install pygifsicle

```

### PyTorch3D

*Note: MAYBE NOT NEEDED FOR GOAT*

```bash
pip install fvcore iopath
git clone [https://github.com/facebookresearch/pytorch3d.git](https://github.com/facebookresearch/pytorch3d.git)
cd pytorch3d
export TORCH_CUDA_ARCH_LIST="8.6"
export FORCE_CUDA=1 
pip install -v -e . --no-build-isolation # 20-30m
cd ..

```

> did not install pinoccio, its for hw

### Detectron 2

```bash
pip install pycocotools termcolor tabulate cloudpickle tensorboard # needs build
pip install omegaconf hydra-core packaging
pip install lvis --no-deps
pip install cycler kiwisolver # for lvis
pip install mss nltk fasttext


pip install src/third_party/detectron2 --no-build-isolation

```

> Ran init_container.sh cmds except habitat-baselines
> install ros noetic

### Unused Packages

**TRY WITHOUT THEM:** `pytest, click, pynput, pinocchio, pybullet, pytorch3d`

---

## ROS Packages Installation

### Librealsense (Realsense SDK)

```bash
sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
sudo add-apt-repository "deb [https://librealsense.intel.com/Debian/apt-repo](https://librealsense.intel.com/Debian/apt-repo) $(lsb_release -cs) main" -u
sudo apt-get install librealsense2-utils
sudo apt-get install librealsense2-dev
git clone [https://github.com/IntelRealSense/librealsense.git](https://github.com/IntelRealSense/librealsense.git)
# ref: [https://docs.ros.org/en/iron/p/librealsense2/user_docs/installation_jetson.html](https://docs.ros.org/en/iron/p/librealsense2/user_docs/installation_jetson.html)
sudo apt-get install git libssl-dev libusb-1.0-0-dev libudev-dev pkg-config libgtk-3-dev -y
./scripts/setup_udev_rules.sh  
mkdir build && cd build  
cmake .. -DBUILD_EXAMPLES=true -DCMAKE_BUILD_TYPE=release -DFORCE_RSUSB_BACKEND=false -DBUILD_WITH_CUDA=true && make -j$(($(nproc)-1)) && sudo make install

```

### Realsense ROS

```bash
sudo apt install ros-noetic-ddynamic-reconfigure
mkdir -p realsense_ws/src && cd realsense_ws/src
git clone [https://github.com/IntelRealSense/realsense-ros.git](https://github.com/IntelRealSense/realsense-ros.git)
cd realsense-ros/
git checkout `git tag | sort -V | grep -P "^2.\d+\.\d+" | tail -1`
cd ..
catkin_init_workspace
cd ..
catkin_make clean
catkin_make -DCATKIN_ENABLE_TESTING=False -DCMAKE_BUILD_TYPE=Release
catkin_make install
cd realsense_ws/src && git clone -b noetic [https://github.com/ros-perception/vision_opencv.git](https://github.com/ros-perception/vision_opencv.git)
cd .. && catkin_make

```

### Livox Lidar

```bash
sudo apt install ros-noetic-pcl-ros
# Follow ref: [https://blog.us.fixstars.com/trying-out-the-livox-mid-360-with-ros1-ros2/](https://blog.us.fixstars.com/trying-out-the-livox-mid-360-with-ros1-ros2/)
$ cd $HOME
$ git clone [https://github.com/Livox-SDK/Livox-SDK2](https://github.com/Livox-SDK/Livox-SDK2) -b 1.0.1
$ cd $HOME/Livox-SDK2/ && mkdir build && cd build
$ cmake .. && make -j
$ sudo make install
$ mkdir $HOME/livox_ws/src -p && cd $HOME/livox_ws/src
$ git clone [https://github.com/Livox-SDK/livox_ros_driver2](https://github.com/Livox-SDK/livox_ros_driver2) -b 1.1.1
$ cd livox_ros_driver2
$ ./build.sh ROS1
$ catkin_make -DROS_EDITION=ROS1

```

> Refer to the link above for config updates and launch file updates

### Direct Lidar Inertial Odometry (DLIO)

```bash
mkdir -p dlio_ws/src
cd dlio_ws/src
sudo apt install libomp-dev libpcl-dev libeigen3-dev
git clone [https://github.com/vectr-ucla/direct_lidar_inertial_odometry.git](https://github.com/vectr-ucla/direct_lidar_inertial_odometry.git)
cd ..
catkin_make

```

### Scout Robot Packages

Refer readme at https://github.com/agilexrobotics/scout_ros
