#  MRSS Go1 Motion Package
This repository contains the skeleton to write high level motion planning code for the 2023 Montreal Robotics Summer School (MRSS).

This code should be used as a package in a [TagSLAM MRSS](https://github.com/sachaMorin/tagslam_root/blob/master/README-MRSS.md) workspace. For MRSS, we will
provide desktops where TagSLAM is already installed. You can also install the TagSLAM MRSS stuff on your own machine.

## Installation Instructions
First make sure to fork this repository and **RENAME IT** to some ```$TEAM_NAME```.

Now clone your package in tagslam_root:
```shell
cd ~/tagslam_root/src
git clone $YOUR_FORK_URL
```

Your first commit should be to replace all instances of ```mrss_motion``` with ```mrss_motion_pub_1``` in the following files:
- ```tagslam_root/src/mrss_motion_pub_1/package.xml```
 - ```tagslam_root/src/mrss_motion_pub_1/CMakeLists.txt```
 - ```tagslam_root/src/mrss_motion_pub_1/src/launch/motion_planning.launch```

Once this is done, build your package

```shell
cd ~/tagslam_root
catkin_make
```
## Usage
In any terminal where you need to use your motion planning package, you should run
```shell
source ~/tagslam_root/devel/setup.bash
```

Now make sure that the main [TagSLAM MRSS launch file](https://github.com/sachaMorin/tagslam_root/blob/master/README-MRSS.md) is running in another terminal:

```shell
roslaunch tagslam mrss_laptop.launch rviz:=1
```

Then you can launch motion planning with
```shell
roslaunch mrss_motion_pub_1 motion_planning.launch twist:=1 rl_policy:=0
```
Setting ```twist:=1``` will launch the Unitree controller. The controller will listen to the ```cmd_vel``` topic and track
the velocities published by the planner node.

Setting ```rl_policy:=1``` will launch an RL controller. Make sure to update your policy path in ```tagslam_root/src/mrss_motion_pub_1/nodes/rl_policy.py```.
Before launching your RL policy, you should always kill the sports mode on the robot. You can do this by calling the
̀```./kill-sport-mode.sh``` located under ```~/go1-rl/unitree-api-wrapper```.


**Do not use both twist and rl_policy at the same time.**

## Workshop
The launch file can spin up 4 ROS nodes:
- The **map_broadcaster** node.
- The **planner** node. **FIX ME!**
- The **goal_broadcaster** node.
- The **rl_policy** node.

We will aim to add basic goal reaching capabilities to the **planner** node.

Following the appropriate lecture, we will attempt to add obstacle avoidance capabilities to the **planner** node following a lecture on planning and motion planning.

All relevant source code is under ```tagslam_root/src/mrss_motion_pub_1/src/nodes```.


## map_broadcaster node
**UPDATE: The map_boadcaster node is now complete. You are free to hack it if you want, but it should provide all
the transforms you need to write your planner.**

 TagSLAM estimates and publishes all sorts of transforms to the ```tf``` topic. The **map_broadcaster** node should read relevant transforms (e.g., the goal, obstacles) and save them to a Python Dict. 
 
The Dict is then
serialized and streamed to a UDP port and to the ```/map``` ROS topic. RL stuff will likely use the UDP port while
the **planner** will rely on the ```/map``` topic.

The **map_broadcaster** currently publishes a single dummy transform. Add more transforms! You may want to have a look at the [tf documentation](http://wiki.ros.org/tf#:~:text=tf%20is%20a%20package%20that,any%20desired%20point%20in%20time.).

Here are some useful commands. This one allows to visualize the transform tree and undertand the objects you can query. 
You can safely igonore the ```camera_link``` and ```uodom``` trees, as well as the ```trunk``` subtree:
```shell
rosrun rqt_tf_tree rqt_tf_tree 
```
You want to print out the ```/map```  topic
```shell
rostopic echo /map
```


## planner node

This node should read the map from the ```/map``` topic, run some planning computations and publish twist commands (linear and angular velocities) to the ```/cmd_vel``` topic.

You should begin by understanding how to move the robot around. You may try 
- Moving the robot forward;
- Having the robot rotate in place;
- Having the robot walk in a circle.

Following this, you should start using the ```map``` to move the robot the the ```goal``` pose.


**Please always run low velocities when first testing your code (e.g, 0.15). The robot does not need to sprint!**

If you need to visualize your twist commmands, this command may be useful
```shell
rostopic echo /cmd_vel
```

## rl_policy node
You should update the path to your policy in ```rl_policy.py```.  If you need to run the RL policy
in isolation, you can run the node
```shell
rosrun mrss_motion_pub_1 rl_policy.py
```
and in another terminal, stream velocity commands, for example
```shell
rostopic pub /cmd_vel -r 30 geometry_msgs/Twist "linear:
  x: 0.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0" 
```

## goal_broadcaster node
Currently, the goal position can be set in front of any one of the "boards" (the tiles or posters with 4 static april tags).
By default, the goal will be `board1`. A parameter controls the goal object and can be changed with

```shell
rosparam set /mrss_motion/goal_object board0
rosparam set /mrss_motion/goal_object board1
rosparam set /mrss_motion/goal_object board2
rosparam set /mrss_motion/goal_object board3
```
This node should work out of the box.

## ROS Bag
If you have ROS on your computer, the main [TagSLAM MRSS](https://github.com/sachaMorin/tagslam_root/blob/master/README-MRSS.md) README contains
instructions to playback all topics. This will allow you to work on your code even if you do not currently have 
access to the robot.


source local-ros.sh