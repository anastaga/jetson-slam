##Check OPENCV version: dpkg -l | grep libopencv
Download SYNAPTIC Package manager to remove OpenCV 4.1. If it doesnt work after the installation of ros-melodic. Then use Synaptic to install the necessary dependancies.

##Install Ceres Solver
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

	
##Install Sophus
	git clone http://github.com/strasdat/Sophus.git
	cd Sophus
	git checkout a621ff
	/Go to so2.cpp and change -> unit_comples_.real() = 1; and unit_comples_.ima() = 0; 
with -> unit_complex_ = std::complex<double>(1,0);/
	mkdir build
	cd build
	cmake ..
	make 
	sudo make install

##Install Ros Melodic
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


*(FIX)THERE WILL BE AN ERROR ON the next CATKIN_MAKE:
	sudo gedit /opt/ros/melodic/share/cv_bridge/cmake/cv_bridgeConfig.cmake 

change this line:
	set(_include_dirs "include;/usr/include;/usr/include/opencv") 
to
	set(_include_dirs "include;/usr/include;/usr/include/opencv4")
	


##Install realsense-ros
	#Step 1: Install RealSense™ SDK 2.0 https://github.com/IntelRealSense/librealsense/blob/master/doc/installation_jetson.md
	#Step 2: https://github.com/IntelRealSense/realsense-ros/tree/ros1-legacy#step-2-install-intel-realsense-ros-from-sources

	Step1:
sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
sudo add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" -u
sudo apt-get install librealsense2-utils
sudo apt-get install librealsense2-dev
	Step2:
cd ~/catkin_ws/src
git clone https://github.com/IntelRealSense/realsense-ros.git
cd realsense-ros/
git checkout `git tag | sort -V | grep -P "^2.\d+\.\d+" | tail -1`
cd .. 
cd ..

catkin_make clean
catkin_make -DCATKIN_ENABLE_TESTING=False -DCMAKE_BUILD_TYPE=Release
catkin_make install



##Install Vins-RGBD-Fast
	git clone https://github.com/jianhengLiu/VINS-RGBD-FAST.git



##Download Config file from midres github repo and replace
	Vins-RGBD: 
	Vins-RGBD-FAST: /home/midres/catkin_ws/src/VINS-RGBD-FAST/config/realsense/vio.yaml 
	

##Commands to run

roslaunch vins_estimator vins_rviz.launch 
roslaunch vins_estimator realsense_color.launch 
roslaunch realsense2_camera rs_camera.launch enable_accel:=true enable_gyro:=true align_depth:=true unite_imu_method:=linear_interpolation color_width:=640 color_height:=480 color_fps:=15 depth_width:=640 depth_height:=480 depth_fps:=15









