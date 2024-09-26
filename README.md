# CLIK-Franka-Panda
A Franka Emika Panda model is used within the Gazebo simulation environment and three methods of Closed-Loop Inverse Kinematics (CLIK) are implemented using the Pinocchio library.
These methods are: 1. Jacobian transpose method; 2. Pseudoinverse method; 3. Damped least squares method.<br />

### System requirements
Ubuntu 20.04 and ROS Noetic

### Major dependencies

- [Pinocchio](https://github.com/stack-of-tasks/pinocchio)
- [panda-gazebo](https://github.com/rickstaa/panda-gazebo/tree/noetic)
- [franka_ros](https://frankaemika.github.io/docs/index.html)



### Results

Given the same goal pose (position plus orientation), the following images show the 
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
