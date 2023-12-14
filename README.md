# Autonomous UAV with RGBD-Inertial Slam and Thermal Camera Human Detection in Jetson Xavier NX

## Overview
This project integrates advanced SLAM and thermal imaging systems into UAVs, building upon the VINS_RGBD SLAM system, and adapting it for enhanced navigation and real-time image processing.

## Test Flight Results
![Midres Test Flight](https://github.com/anastaga/midres-jetson/blob/main/MIDRES-test-flight.gif?raw=true)

[🎥 Watch the Full Video Here!](https://www.youtube.com/watch?v=JHDNFKczD44)

## Procedure

### Advanced Simulation
- Setup Quadcopter Simulation in Gazebo [Hector_Quadrotor](https://github.com/tu-darmstadt-ros-pkg/hector_quadrotor).
- Integrate Realsense d435i camera using the Reasense SDK and a custom thermal camera plugin in simulation.


### Neural Network Training and Optimization
- Employ pretrained Yolov8 for RPN, re-trained on ~10,000 thermal images.
- Balance efficiency and accuracy for real-time human detection.

### Hardware Implementation and Calibration
- Transition to real-world application with Jetson Xavier NX.
- Integrate COIN417G2 thermal camera and Realsense d435i.

### Flight Control and SLAM System Integration
- Connect with Cube Orange flight controller via MAVROS.
- Refine system to send back obstacle data for navigation and receive IMU data.

### Algorithm Customization
- Tailor Yolov8 for real-time processing on Jetson Xavier NX.

### Flight Test
- Test flights showcased system functionality and efficiency in complex environments.


## TO INSTALL THE BASE SLAM WE USED, ALONG WITH THE REALSENSE D435i CAMERA INTEGRATION ON JETSON XAVIER NX


## Install Ros Melodic
	sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
	sudo apt install curl # if you haven't already installed curl
	curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
	sudo apt update
	sudo apt install ros-melodic-desktop
	sudo apt install ros-melodic-cv-bridge
	sudo apt install ros-melodic-ddynamic-reconfigure
	sudo apt install ros-kinetic-pcl-ros
	echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
	source ~/.bashrc
	sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential
	sudo apt install python-rosdep
	sudo rosdep init
	rosdep update
	echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
	
**Download SYNAPTIC Package manager to remove OpenCV 4.1**. If it doesnt work after the installation of ros-melodic. Then use Synaptic to install the necessary dependancies.
Check OPENCV version: ```dpkg -l | grep libopencv```

## Install Ceres Solver
	sudo apt-get install libgoogle-glog-dev libgflags-dev	
	sudo apt-get install libatlas-base-dev
	sudo apt-get install libeigen3-dev
	sudo apt-get install libsuitesparse-dev
	wget http://ceres-solver.org/ceres-solver-2.1.0.tar.gz
	mkdir ceres-bin
	cd ceres-bin
	cmake ../ceres-solver-2.1.0
	make -j6 
	make test  
	sudo make install

	
## Install Sophus
	git clone http://github.com/strasdat/Sophus.git
	cd Sophus
	git checkout a621ff
 
	
_Go to so2.cpp and change -> unit_comples_.real() = 1; and unit_comples_.ima() = 0; 
with -> unit_complex_ = std::complex<double>(1,0);_

	mkdir build
	cd build
	cmake ..
	make 
	sudo make install

## Install realsense-ros
	Step 1: Install RealSense™ SDK 2.0 (https://github.com/IntelRealSense/librealsense/blob/master/doc/installation_jetson.md)
	Step 2: (https://github.com/IntelRealSense/realsense-ros/tree/ros1-legacy#step-2-install-intel-realsense-ros-from-sources)

## Install realsense-ros steps (code from previous links)
```
sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
sudo add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" -u
sudo apt-get install librealsense2-utils
sudo apt-get install librealsense2-dev
cd ~/catkin_ws/src
git clone https://github.com/IntelRealSense/realsense-ros.git
cd realsense-ros/
git checkout `git tag | sort -V | grep -P "^2.\d+\.\d+" | tail -1`
cd .. 
cd ..
catkin_make clean
catkin_make -DCATKIN_ENABLE_TESTING=False -DCMAKE_BUILD_TYPE=Release
catkin_make install
```

## Install Vins-RGBD
	git clone https://github.com/jianhengLiu/VINS-RGBD.git

	

## Run (each command should be run in a different terminal. Do not forget to connect the Realsense camera)
```
roslaunch vins_estimator vins_rviz.launch 

roslaunch vins_estimator realsense_color.launch 

roslaunch realsense2_camera rs_camera.launch enable_accel:=true enable_gyro:=true align_depth:=true
unite_imu_method:=linear_interpolation color_width:=640 color_height:=480 color_fps:=15 depth_width:=640
depth_height:=480 depth_fps:=15
```

_________________________
# Thermal Camera and Human Presence Detection

Due to the proprietary nature of the thermal camera's APK, we are unable to share the specific code we developed. However, we provide a general outline of our methodology below.

## Process Overview

### 1. Image Acquisition and Conversion
The process begins with acquiring an image from the thermal camera. This image is then converted into a format compatible with OpenCV for further processing.

### 2. ROS Node Initialization
A ROS (Robot Operating System) node is established to initiate the entire process. This node is crucial for managing the image data and interfacing with other system components.

### 3. Utilizing Yolov8 for Object Detection
For human detection, we utilized a pretrained Yolov8 model, specifically its lightweight version, to balance processing efficiency with accuracy. Our model was trained on a dataset of approximately 10,000 thermal images, including both human and non-human subjects, sourced from various online resources.

### 4. Training and Configuration
The training phase was conducted with careful attention to ensure the model configurations were set appropriately for accurate human detection.

### 5. Setting Up the Jetson Xavier NX
A virtual environment was set up on the Jetson Xavier NX, tailored to the specific Python version required for Yolov8. All necessary libraries and requirements were installed, following the guidelines on the [Yolov8 GitHub page](https://github.com/ultralytics/ultralytics).

### 6. ROS Node for Image Processing
We developed a ROS node that subscribes to the Thermal Camera Node. This node processes the thermal image through the Yolov8 network to detect human presence.



_________________________
## Communication Protocols
Guide for connecting to the Flight Controller and exchanging messages for tasks like obstacle avoidance.
(to-do)

## Install Mavros
```
sudo apt-get install ros-${ROS_DISTRO}-mavros ros-${ROS_DISTRO}-mavros-extras ros-${ROS_DISTRO}-mavros-msgs
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
sudo bash ./install_geographiclib_datasets.sh

```
