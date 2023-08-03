# Suhan Robot Model Tools
Code for motion planning and planning scene

## Installation guide
Install dependencies
```sh
sudo apt install ros-$ROS_DISTRO-combined-robot-hw ros-$ROS_DISTRO-trac-ik ros-noetic-moveit
sudo apt install ros-$ROS_DISTRO-nlopt

sudo apt install libnlopt-dev libnlopt-cxx-dev python-numpy

pip install tqdm
```

Install step and running code example
```sh

cd ~/catkin_ws/src/

git clone https://github.com/psh117/suhan_robot_model_tools.git
git clone https://github.com/psh117/gl_depth_sim.git

catkin build gl_depth_sim
catkin build suhan_robot_model_tools

source ../devel/setup.bash
echo export PYTHONPATH=\$PYTHONPATH:~/catkin_ws/src/suhan_robot_model_tools >> ~/.bashrc
```
```sh
roslaunch panda_moveit_config demo.launch
```
Shut down after open rviz.

```sh
roscore
python examples/planning_scene_to_vis_sim.py
```

## Code Instruction

```sh
1. collision_checker
   Use for calculating planning scene collision
   
2. eigen_tools
   c++ library for linear algebra and vector calculus
   
3. python
   main function for planning scene
   
4. trac_ik_adapter
   IK solver Library
   
5. suhan_robot_model_tools
```
