#include <ros/ros.h>
#include <franka_msgs/FrankaState.h>
#include <Eigen/Dense>


// The O_T_EE field in the franka_msgs/FrankaState message represents the homogeneous transformation matrix that describes 
// the pose of the end-effector relative to the base frame of the robot. This matrix is a 4x4 matrix flattened into a 1x16 array in row-major order. 
// Rotation Matrix: The first 3x3 submatrix (first 9 elements) represents the rotation part of the transformation.
// Translation Vector: The translation vector is the last row excluding the last element 1.
// Homogeneous transformation matrices


void stateCallback(const franka_msgs::FrankaState::ConstPtr &msg)
{
    // Extract the 4x4 transformation matrix from O_T_EE
    Eigen::Matrix4d O_T_EE;
    for (int i = 0; i < 4; ++i)
    {
        for (int j = 0; j < 4; ++j)
        {
            O_T_EE(i, j) = msg->O_T_EE[i * 4 + j];
        }
    }
    // Extract the rotation matrix (top-left 3x3 part)
    Eigen::Matrix3d rotation_matrix = O_T_EE.block<3, 3>(0, 0);
    // Extract the translation vector (down-left 1x3 part)
    Eigen::Vector3d translation_vector = O_T_EE.block<1, 3>(3, 0);
    // Print the rotation matrix
    ROS_INFO_STREAM("Rotation Matrix:\n"
                    << rotation_matrix);
    // Print the translation vector
    ROS_INFO_STREAM("Translation Vector:\n" << translation_vector.transpose());
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "end_effector_orientation_listener");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/franka_state_controller/franka_states", 10, stateCallback);
    // Check if the subscriber is correctly initialized
    if (sub)
    {
        ROS_INFO("Subscriber successfully initialized.");
    }
    else
    {
        ROS_ERROR("Failed to initialize subscriber.");
    }
    ros::spin();
    return 0;
}
