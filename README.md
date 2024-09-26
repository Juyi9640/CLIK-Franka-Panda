# CLIK-Franka-Panda
A Franka Emika Panda model is used within the Gazebo simulation environment and three methods of Closed-Loop Inverse Kinematics (CLIK) are implemented using the Pinocchio library.
These methods are: 1. Jacobian transpose method; 2. Pseudoinverse method; 3. Damped least squares method.<br />

### System requirements
[Ubuntu 20.04][1] and [ROS Noetic][2]

[1]: https://releases.ubuntu.com/focal/                "Ubuntu 20.04"
[2]: http://wiki.ros.org/noetic/Installation/Ubuntu    "ROS Noetic"

### Major dependencies

- [Pinocchio](https://github.com/stack-of-tasks/pinocchio)
- [panda-gazebo](https://github.com/rickstaa/panda-gazebo/tree/noetic)
- [franka_ros](https://frankaemika.github.io/docs/index.html)
- [Gazebo](https://gazebosim.org/home)
- [RViz](http://wiki.ros.org/rviz)

## Implementation details

### Posture generation
We define a custom pose (which contains end-effector’s position and orientation) for the
robot by dragging the marker (that locates at robot’s end-effector) in RViz to a specific pose and we implement a ROS node to extract the orientation and position values of this pose at real-time.
```bash
source devel/setup.bash
roslaunch franka_gazebo panda.launch controller:=cartesian_impedance_example_controller     rviz:=true
```
The command above opens Gazebo and RViz at the same time, we can then drag the marker to move the robot:

<p align="center">
  <img src="pictures/rviz pose.png" width="350" title="hover text">
</p>

Now open another terminal and input:
```bash
source devel/setup.bash
rosrun panda_clik_control end_effector_orientation
```
The orientation matrix and translation vector appears as follows:
<p align="center">
  <img src="pictures/rviz pose read.png" width="350" title="hover text">
</p>



### Results

Given the same goal pose (position and orientation extracted from RViz), the following images show the 
qualitative comparison between the results: <br />
(left: Jacobian transpose; middle: Pseudoinverse; right: Damped least squares)

<div align="center">
  <p style="text-align:center;">IKs with the first goal pose</p>

  <img src="gif folder/11.gif" width="300" alt="First GIF" />
  <img src="gif folder/21.gif" width="300" alt="Second GIF" />
  <img src="gif folder/31.gif" width="300" alt="Third GIF" />

  <p style="text-align:center;">IKs with the second goal pose</p>

  <img src="gif folder/12.gif" width="300" alt="First GIF" />
  <img src="gif folder/22.gif" width="300" alt="Second GIF" />
  <img src="gif folder/32.gif" width="300" alt="Third GIF" />

</div>
