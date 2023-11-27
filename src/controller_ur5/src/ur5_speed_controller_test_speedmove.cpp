#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose.h"
#include "control_msgs/JointTrajectoryControllerState.h"
#include "control_msgs/FollowJointTrajectoryActionGoal.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "trajectory_msgs/JointTrajectoryPoint.h"
#include "std_msgs/Float64MultiArray.h"
#include <cmath>
#include <iostream>
#include <sstream>
#include "std_srvs/Empty.h"
#include <eigen3/Eigen/Dense>
#include <stdio.h>
#include <trac_ik/trac_ik.hpp>
#include "control_function_ur5.h"
#include "Utils.h"

using namespace Eigen;
using namespace std;


int main(int argc, char **argv)
{
    //choose the time step for ros
    double delta_t = 0.05;
    //choose the tintegration time for the next pos 
    double integrationTime = 0.05;
    // choose the tolerance to the new joints
    float tol_mse =0.2;
    //choose the tolerance and time for the inverse kinematic
    float err_ik =0.001;
    float t_ik = 0.05;

        
    //choose intial pose
    vector<double> intialPos={0.3,0,0.8};
    vector<double> intialState;


    
    //Initialisation of the Ros Node (Service, Subscrber and Publisher)
    ros::init(argc, argv, "Ds");
    ros::NodeHandle Nh_;
    //ros::ServiceClient FK      = Nh_.serviceClient<iiwa_tools::GetFK>("iiwa/iiwa_fk_server");
    ros::Publisher chatter_pub = Nh_.advertise<trajectory_msgs::JointTrajectory>("/eff_joint_traj_controller/command", 1000);

    ros::Publisher pub_pos     = Nh_.advertise<geometry_msgs::Pose>("/iiwa/ee_info/Pose", 1000);
    ros::Publisher pub_speed   = Nh_.advertise<geometry_msgs::Twist>("/iiwa/ee_info/Vel", 1000);
    //ros::ServiceClient client  = Nh_.serviceClient<iiwa_tools::GetJacobian>("/iiwa/iiwa_jacobian_server");

    ros::Rate loop_rate(1/delta_t);

    // //Define object position and speed
    // ActualState actualState(FK);
    // NextState nextState(err_ik,t_ik);

    // ros::Subscriber sub =  Nh_.subscribe("/eff_joint_traj_controller/state", 1000, &ActualState::CounterCallback, &actualState); //control_msgs/JointTrajectoryControllerState
    // ros::Subscriber sub_DS =  Nh_.subscribe("/passive_control/vel_quat", 1000, &NextState::poseCallback, &nextState);
   float test = 0;
    while (ros::ok()){
        // Create a JointTrajectoryControllerState message
        // Create a std_msgs/MultiArray message
        // Create a trajectory_msgs/JointTrajectory message
        trajectory_msgs::JointTrajectory msg;

        // Set the joint names for the robot (replace with your joint names)
        msg.joint_names = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};

        // Create a trajectory point (example: a single waypoint)
        trajectory_msgs::JointTrajectoryPoint point;

        // Set positions, velocities, accelerations, and effort (if needed) for the trajectory point
        point.positions = {test, 0 , 0 , 0,0,0}; // Set desired positions for each joint
        point.velocities = {1, 1, 0.0, 0.0, 0.0, 0.0}; // Set desired velocities (if applicable)
        point.accelerations = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // Set desired accelerations (if applicable)
        point.effort = {0,0,0,0,0,0}; // Set desired efforts (if applicable)

        // Set the time from the start for this trajectory point (in seconds)
        point.time_from_start = ros::Duration(3.0); // Set the desired time for this waypoint (1 second in this case)
        test = test+test;
        // Add the trajectory point to the trajectory message
        msg.points.push_back(point);
        // Publish the filled JointTrajectoryControllerState message
        chatter_pub.publish(msg); 
        ros::spinOnce(); // Allow the message to be published
    }
    return 0;    
}
