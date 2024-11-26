# opr_ros
ROS package for CIT's Open Platform Robot. Tested on ROS Noetic.

## Installation
1. install ros
2. create ROS workspace
```bash
sudo apt install python3-catkin-tools
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin init
echo 'source ~/catkin_ws/devel/setup.bash' >> ~/.bashrc
echo 'source `catkin locate --shell-verbs`' >> ~/.bashrc
```
3. clone repository
```bash
cd ~/catkin_ws/src
git clone https://github.com/jsupratman13/opr_ros.git
```
4. install dependencies
```bash
wstool init .
wstool merge opr_ros/.rosinstall
wstool update
rosdep install --from-paths . -iry
```
5. build and source
```bash
cd ~/catkin_ws
catkin build
catkin source
```

## Usage
* View URDF model on RViz
```
roslaunch opr_bringup display_xacro.launch
```
* Gazebo Simulation
```
roslaunch opr_bringup gazebo.launch
```
* Real Robot
```
roslaunch opr_bringup sustaina.launch
```

## Package Description
* opr_bringup
  * collection of launch files to bring up robot or gazebo
* opr_button
  * external start stop button node
  * upload firmware with `pio run -t upload`
* opr_description
  * collection of robot model's URDF
* opr_gazebo
  * collection of gazebo worlds/models
* opr_imu
  * collection of robot's imu node
    * ICM42688_node's firmware can be found [here](https://github.com/SUSTAINA-OP/IMU-Measurement-and-Transmission-Module)
* opr_ros
  * metapackage
