

#include <iostream>
#include <pinocchio/fwd.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/utils/timer.hpp>
#include <Eigen/Dense>
#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <vector>
#include <algorithm>
#include <cmath>

// global variable oMdes
// initialize pose
pinocchio::SE3 oMdes(Eigen::Matrix3d::Identity(), Eigen::Vector3d(0.307129, 0.000127757, 0.487072));

// Function to clamp a value within given limits
double clamp(double value, double min, double max)
{
    return std::max(min, std::min(value, max));
}

// clamp joint positions
std::vector<double> clampJointPositions(const std::vector<double> &joint_positions_in_rad)
{
    // Joint limits in radians
    const std::vector<double> q_min = {-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973};
    const std::vector<double> q_max = {2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973};

    std::vector<double> clamped_joint_pos_in_rad;

    for (size_t i = 0; i < joint_positions_in_rad.size(); ++i)
    {
        double clamped_pos_rad = clamp(joint_positions_in_rad[i], q_min[i], q_max[i]);
        clamped_joint_pos_in_rad.push_back(clamped_pos_rad);
    }
    return clamped_joint_pos_in_rad;
}

void selectPose(char message)
{
    if (message == '1')
    {
        Eigen::Matrix3d rotation_matrix1;
        rotation_matrix1 << 0.339197, -0.721834, 0.603226,
            -0.357597, -0.692045, -0.627039,
            0.870095, -0.00302198, -0.492875;

        Eigen::Vector3d position1;
        position1 << 0.288347, 0.25953, 0.632161;

        // Desired pose (position & orientation)
        oMdes.rotation() = rotation_matrix1.transpose();
        oMdes.translation() = position1;
    }
    else if (message == '2')
    {
        Eigen::Matrix3d rotation_matrix2;
        rotation_matrix2 <<  0.529746, -0.810064,   0.25129,
            -0.714637,  -0.58588, -0.382124,
            0.456779, 0.0228478, -0.889287;

        Eigen::Vector3d position2;
        position2 << 0.0713315, -0.550705,  0.143914;

        // Desired pose (position & orientation)
        oMdes.rotation() = rotation_matrix2.transpose();
        oMdes.translation() = position2;
    }
    else if (message == '3')
    {
        // near singularity
        Eigen::Matrix3d rotation_matrix3;
        rotation_matrix3 << -0.438744, 0.501727, 0.745489,
            -0.283179, -0.86453, 0.415184,
            0.852824, -0.0289481, 0.521396;

        Eigen::Vector3d position3;
        position3 << 0.164124, -0.47472, 0.899756;

        // Desired pose (position & orientation)
        oMdes.rotation() = rotation_matrix3.transpose();
        oMdes.translation() = position3;
    }
    else if (message == '4')
    {
        // go back to initial pose
        Eigen::Matrix3d rotation_matrix4;
        rotation_matrix4 << 0.999898, 0.000624663, 0.0135856,
            0.000622934, -0.99999, 0.000131514,
            0.0135858, -0.00012304, -0.999908;

        Eigen::Vector3d position4;
        position4 << 0.307129, 0.000127757, 0.487072;

        // Desired pose (position & orientation)
        oMdes.rotation() = rotation_matrix4.transpose();
        oMdes.translation() = position4;
    }
    else
    {
        std::cout << "Invalid input, try again in the next loop. " << std::endl;
    }
}

int main(int argc, char **argv)
{
    // ros node name: panda_clik_control
    ros::init(argc, argv, "panda_clik_control");
    ros::NodeHandle nh;
    ros::Publisher traj_pub = nh.advertise<trajectory_msgs::JointTrajectory>("/panda_arm_trajectory_controller/command", 10);

    // Give some time to the publisher to connect to subscribers
    ros::Duration(1.0).sleep();

    // Load the robot model from URDF with panda hand
    std::string urdf_path = "/path/to/your/urdf/panda_with_hand.urdf";

    pinocchio::Model model;
    pinocchio::urdf::buildModel(urdf_path, model);
    pinocchio::Data data(model);

    // Joint ID
    int JOINT_ID = model.getJointId("panda_joint7");

    // Frame ID
    int frame_ID = model.getFrameId("panda_hand_tcp_joint");

    Eigen::Matrix3d rotation_initial;
    rotation_initial << 0.999898, 0.000624663, 0.0135856,
        0.000622934, -0.99999, 0.000131514,
        0.0135858, -0.00012304, -0.999908;
    oMdes.rotation() = rotation_initial;

    // Initial joint configuration
    // Eigen::VectorXd q = pinocchio::neutral(model);
    // Instead of initialize q in neutral position, use the preset from panda.launch
    // Later can be optimized with subscriber to panda_joints and set it automatically
    // Fitted joint configuration
    Eigen::VectorXd q(9);
    q << 0.0, -0.785398163, 0.0, -2.35619449, 0.0, 1.57079632679, 0.785398163397, 0, 0;

    // Control loop parameters
    const double eps = 1e-4;
    const int IT_MAX = 1000;
    // const double DT = 1e-1;
    // const double damp = 1e-6;

    // pre-allocate Jabocian matrix
    pinocchio::Data::Matrix6x J(6, model.nv);
    J.setZero();

    bool success = false;
    // Define error term as 6x1 vector
    typedef Eigen::Matrix<double, 6, 1> Vector6d;
    Vector6d err;
    // Define joint velocities
    Eigen::VectorXd deltaTheta(model.nv);

    // choose a pose
    while (true)
    {

        for (int i = 0;; i++)
        {

            pinocchio::forwardKinematics(model, data, q);
            pinocchio::updateFramePlacement(model, data, frame_ID);
            // dMi: the transformation between the desired pose and current pose.
            const pinocchio::SE3 dMi = oMdes.actInv(data.oMf[frame_ID]);
            // use MOTION object to represent error in SO(3) as a 6D vector
            err = pinocchio::log6(dMi).toVector();
            if (err.norm() < eps)
            {
                success = true;
                break;
            }
            if (i >= IT_MAX)
            {
                success = false;
                break;
            }
            pinocchio::computeJointJacobian(model, data, q, JOINT_ID, J);
            pinocchio::Data::Matrix6 JJt; // JJt = J*J_T (6*7*7*6 = 6*6)

            JJt.noalias() = J * J.transpose();
            // JJt.diagonal().array() += damp;
            deltaTheta.noalias() = -J.transpose() * JJt.ldlt().solve(err);
            // Update the q with v*DT
            q = pinocchio::integrate(model, q, deltaTheta);
            if (!(i % 10))
            {
                std::cout << i << ": error + " << err.transpose() << std::endl;
            }

            // To convert joint positions q from degrees to radians while considering the panda joint limits
            std::vector<double> joint_positions_in_radian{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
            for (size_t j = 0; j < joint_positions_in_radian.size(); ++j)
            {
                joint_positions_in_radian[j] = q[j];
            }
            std::vector<double> clamped_joint_positions_in_radian = clampJointPositions(joint_positions_in_radian);

            trajectory_msgs::JointTrajectory traj;
            // synchronize timestamps
            traj.header.stamp = ros::Time::now();
            traj.joint_names = {"panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4",
                                "panda_joint5", "panda_joint6", "panda_joint7"};

            trajectory_msgs::JointTrajectoryPoint point;
            point.positions.resize(clamped_joint_positions_in_radian.size());
            // Send the new joint configuration to the robot
            for (int j = 0; j < clamped_joint_positions_in_radian.size(); ++j)
            {
                point.positions[j] = clamped_joint_positions_in_radian[j];
            }
            // the execution time for a trajectory
            point.time_from_start = ros::Duration(0.1);

            traj.points.push_back(point);

            traj_pub.publish(traj);

            // Give some time for trajectory controller to execute
            ros::Duration(0.11).sleep();
        }

        if (success)
        {
            std::cout << "Convergence achieved!" << std::endl;
        }
        else
        {
            std::cout << "\nWarning: the iterative algorithm has not reached convergence to the desired precision" << std::endl;
        }
        std::cout << "\nresult: " << q.transpose() << std::endl;
        std::cout << "\nfinal error: " << err.transpose() << std::endl;

        char ch1 = 'n';
        std::cout << "Start again, [y/n] ? " << std::endl;
        std::cin >> ch1;
        if (ch1 == 'Y' || ch1 == 'y')
        {
            char ch2 = '4';
            std::cout << "Choose a pose [1/2/3] or go back to initial pose [4]: " << std::endl;
            std::cin >> ch2;

            selectPose(ch2);
            continue;
        }
        else
            break;
    }
    return 0;
}

