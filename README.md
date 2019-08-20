# cone slam

orb slam 2 + yolo + dbscan

## Prerequisites

* C++11 or C++0x Compiler		
* [ROS](http://wiki.ros.org/ROS/Installation)
* [OpenCV 3](https://docs.opencv.org/master/d7/d9f/tutorial_linux_install.html) - no need if installing ros
* [Pangolin](https://github.com/stevenlovegrove/Pangolin)
* Eigen 3
```
sudo apt install libeigen3-dev
```
* BLAS and LAPACK (you probably have them)
```
sudo apt-get install libblas-dev
sudo apt-get install liblapack-dev
```
* NumPy, SciPy, scikit-learn

## Install

Open a terminal and navigate to formula workspace or to where you want to create one.
Run:
```
mkdir formula_ws
cd formuls_ws
mkdir src
cd src
git clone https://github.com/tomnorman/cone_slam.git
git clone https://github.com/yardenshap/custom_msgs.git
# compile YOLO:
git clone https://github.com/AlexeyAB/darknet.git
cd darknet
gedit Makefile
# change those params:
## GPU=1
## CUDNN=0 #if you need
## CUDNN_HALF=0 #if you need
## LIBSO=1
# save and exit
make
mv libdarknet.so libdarklib.so #renaming
mv libdarklib.so ../cone_slam/orb_slam2/Thirdparty/darknet/lib #move so we can compile orb slam2
cd ..
./cone_slam/orb_slam2/build.sh
catkin build
echo 'source ~/formula_ws/devel/setup.bash' >> ~/.bashrc
exit
```

## Parameters

In order to use cone slam you need to change some parameters.

### ORB SLAM

All config files are in:
```
formula_ws/src/cone_slam/orb_slam2/ros/cfg
```

#### <ins>YOLO config, weights and threshold</ins>

Open (with text editor): **mono_slam.yaml or stereo_slam.yaml**. 
Search for 'Yolo.Config' and 'Yolo.Weights' and change
```
/home/nvidia/
```
to your system path.
You can also change 'Yolo.Thresh' - threshold for classification.

#### <ins>Camera calibration and orb slam parameters</ins>

Open (with text editor): **mono_slam.yaml or stereo_slam.yaml**.
Change what you need/want.

#### <ins>ROS</ins>

Open (with text editor): **ros_params.yaml**.
Change what you need/want.

### Cones Map
Open (with text editor): **formula_ws/src/cone_slam/cones_map/cfg/ros_params.yaml**.
Change what you need/want.

## Run
```
# slam node
## stereo
roslaunch orb_slam2 stereo_slam.launch
## mono
roslaunch orb_slam2 mono_slam.launch
# dbscan node
roslaunch cones_map map.launch
# debug node
roslaunch debug_unit debug.launch
```

## Cameras

* [LeadSense ](http://leadsense.ilooktech.com/product?lang=en)
* [MYNT EYE Standard](https://www.mynteye.com/products/mynt-eye-stereo-camera)

## Miscellaneous

* [g2o crashing solution](https://github.com/raulmur/ORB_SLAM2)
* [documented orb_slam2](https://github.com/AlejandroSilvestri/ORB_SLAM2-documented)
* [how to write README.md](https://gist.github.com/PurpleBooth/109311bb0361f32d87a2)

## Acknowledgments

* [ORB_SLAM2](https://github.com/raulmur/ORB_SLAM2)
* [darknet](https://github.com/AlexeyAB/darknet)

