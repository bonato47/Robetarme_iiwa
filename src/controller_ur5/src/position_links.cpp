
#include "pinocchio/fwd.hpp"
#include "ros/ros.h"
#include "geometry_msgs/PoseArray.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose.h"
#include "std_msgs/Float64MultiArray.h"
#include "control_msgs/JointTrajectoryControllerState.h"
#include "control_msgs/FollowJointTrajectoryActionGoal.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "trajectory_msgs/JointTrajectoryPoint.h"
#include "controller_manager_msgs/SwitchController.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
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


class JointStateHandler {
    public:
        vector<double> jointPosition;
        vector<double> jointSpeed;
        bool init_joint;
        JointStateHandler(ros::NodeHandle& nh) : nh_(nh) {
            jointPosition = {0,0,0,0,0,0};
            jointSpeed    = {0,0,0,0,0,0};
            init_joint = true;
        
            sub_ = nh_.subscribe("/ur5/joint_states", 10, &JointStateHandler::jointStateCallback, this);
        }

    void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg) {
        if (!msg->position.empty()) {
                jointPosition = msg->position; // Update the position vector with received positions
                jointSpeed = msg->velocity; // Update the position vector with received positions
                // swwap the position to have each joint in the kinematic order
                swap(jointPosition[0], jointPosition[2]);
                swap(jointSpeed[0], jointSpeed[2]);
                init_joint = false;
                
            } else {
                ROS_WARN("Received joint positions are empty.");
            }
    }
    private:
        ros::NodeHandle nh_; // Member field to store the NodeHandle
        ros::Subscriber sub_;
};



class LinearDS {
private:
    bool doModulation;
    vector<double> attractor;
    vector<vector<double>> eye;
    // Add other necessary members for the obstacles, world, and relevant structures

public:
    vector<string> objectList;

    vector<double> modulatedDS(const vector<double>& xdot_nominal) {

        // Initialize modulation matrix and average obstacle velocity
        vector<vector<double>> M_tot = eye;
        vector<double> meanObstacleVelocity(3, 0.0);

        // Create the array of gamma values from the list world.listOfObstacles
        // for convenience
        int kObst = objectList.size();
        vector<double> gamma(kObst, numeric_limits<double>::infinity());

        for (int k = 0; k < kObst; ++k) {
            // Update gamma values based on obstacle information
            gamma[k] = max(/* update based on obstacle information */, 1.0);
        }

        // // Go over each obstacle, compute the modulation matrix and apply it to
        // // the initial ds velocity xdot_modulated
        // for (int k = 0; k < kObst; ++k) {
        //     // Compute distance function
        //     double weight;

        //     if (kObst > 1) {
        //         // Compute weight based on gamma values
        //         // Update weight based on obstacle information
        //     } else {
        //         weight = 1.0;
        //     }

        //     // Compute modulation matrix
        //     double gammaCorrected = pow(abs(gamma[k]), 1.0 / /* get obstacle's rho value */);
        //     vector<vector<double>> D{
        //         {1.0 - (weight / gammaCorrected), 0.0, 0.0},
        //         {0.0, 1.0 + (weight / gammaCorrected), 0.0},
        //         {0.0, 0.0, 1.0 + (weight / gammaCorrected)}
        //     };

        //     // Compute normal and tangent on ellipse
        //     vector<double> normal = /* compute normal based on obstacle information */;
        //     // Update normal based on obstacle information

        //     // Compute tangent1 and tangent2
        //     vector<double> tangent1{0.0, -normal[2], normal[1]};
        //     vector<double> tangent2 = /* compute tangent2 based on normal and tangent1 */;

        //     vector<vector<double>> E{
        //         {normal[0], tangent1[0], tangent2[0]},
        //         {normal[1], tangent1[1], tangent2[1]},
        //         {normal[2], tangent1[2], tangent2[2]}
        //     };

        //     vector<vector<double>> M = /* compute M based on E and D */;
        //     M_tot = /* compute M_tot based on M and M_tot */;

        //     // Weighted average of the obstacle velocities
        //     for (int i = 0; i < 3; ++i) {
        //         meanObstacleVelocity[i] += weight * /* get obstacle's velocity */;
        //     }
        // }

        // // Compute the modulated xdot
        // vector<double> xdot(3, 0.0);
        // for (int i = 0; i < 3; ++i) {
        //     for (int j = 0; j < 3; ++j) {
        //         xdot[i] += M_tot[i][j] * (xdot_nominal[j] - meanObstacleVelocity[j]);
        //     }
        //     xdot[i] += meanObstacleVelocity[i];
        // }

        // return xdot;        
        return xdot_nominal;
    }
};


int main(int argc, char **argv)
{
    vector<string> tipList = {"shoulder_link", "forearm_link", "wrist_1_link"};
    vector<vector<double>> poseLink;
    ros::init(argc, argv, "position_links");
    ros::NodeHandle Nh_;
    ros::Publisher jointStatePublisher = Nh_.advertise<sensor_msgs::JointState>("Pose_links", 1);
    double delta_t = 0.01;
    ros::Rate loop_rate(1/delta_t);

    JointStateHandler JsHandler(Nh_);
    RobotParameter RobotUr5;



   while (ros::ok()){
        poseLink.clear();
        // Iterate through each link in tipList
        for (const string& link : tipList) {

            vector<double> poseCartActual = RobotUr5.getFK_link(JsHandler.jointPosition, link);
            // Create and publish a sensor_msgs::JointState for each link
            sensor_msgs::JointState jointStateMsg;
            jointStateMsg.header.frame_id = "base";  // Replace with your frame ID
            jointStateMsg.header.stamp = ros::Time::now();
            jointStateMsg.name.push_back(link);

            jointStateMsg.position.push_back(poseCartActual[0]);
            jointStateMsg.position.push_back(poseCartActual[1]);
            jointStateMsg.position.push_back(poseCartActual[2]);           
            jointStateMsg.position.push_back(poseCartActual[3]);
            jointStateMsg.position.push_back(poseCartActual[4]);
            jointStateMsg.position.push_back(poseCartActual[5]);
            jointStateMsg.position.push_back(poseCartActual[6]);

            jointStatePublisher.publish(jointStateMsg);
        }



        // // Print the results for each link
        // for (size_t i = 0; i < tipList.size(); ++i) {
        //     cout << "Pose for " << tipList[i] << ": ";
        //     for (double value : poseLink[i]) {
        //         cout << value << " ";
        //     }
        //     cout << endl;
        // }
        ros::spinOnce(); // Allow the message to be published
        loop_rate.sleep(); 
    }
    cout<<"endl"<<endl;
    return 0;
}