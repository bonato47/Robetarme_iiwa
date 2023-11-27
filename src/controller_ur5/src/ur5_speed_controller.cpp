
#include "pinocchio/fwd.hpp"
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose.h"
#include "std_msgs/Float64MultiArray.h"
#include "control_msgs/JointTrajectoryControllerState.h"
#include "control_msgs/FollowJointTrajectoryActionGoal.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "trajectory_msgs/JointTrajectoryPoint.h"
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
        
            sub_ = nh_.subscribe("/joint_states", 10, &JointStateHandler::jointStateCallback, this);
        }

    void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg) {
        if (!msg->position.empty()) {
                jointPosition = msg->position; // Update the position vector with received positions
                jointSpeed = msg->velocity; // Update the position vector with received positions

                init_joint = false;
            } else {
                ROS_WARN("Received joint positions are empty.");
            }
    }
    private:
        ros::NodeHandle nh_; // Member field to store the NodeHandle
        ros::Subscriber sub_;
};



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
    ros::init(argc, argv, "speed_controler");
    ros::NodeHandle Nh_;

    ros::Publisher chatter_pub = Nh_.advertise<std_msgs::Float64MultiArray>("/joint_group_vel_controller/command", 1000);
    ros::Publisher pub_pos     = Nh_.advertise<geometry_msgs::Pose>("/ur5/ee_info/Pose", 1000);
    ros::Publisher pub_speed   = Nh_.advertise<geometry_msgs::Twist>("/ur5/ee_info/Vel", 1000);
   
    ros::Rate loop_rate(1/delta_t);

    RobotParameter RobotUr5();
    InverseKinematics Ik();
    JointStateHandler Js(Nh_);

    //waiting for the first joint position
    while(Js.init_joint){

        ros::spinOnce();
    } 
    
    //Fk_and_pub(actualState,client,pub_pos,pub_speed);

    //auto Cartpos = Ik.getIK(Js.jointPosition);


    float test = 0;
    while (ros::ok()){


        // chatter_pub.publish(msg); 
        ros::spinOnce(); // Allow the message to be published
    }
    return 0;    

}

    // ros::Subscriber sub =  Nh_.subscribe("iiwa/joint_states", 1000, &ActualState::CounterCallback, &actualState);
    // ros::Subscriber sub_DS =  Nh_.subscribe("/passive_control/vel_quat", 1000, &NextState::poseCallback, &nextState);

// ------......--------   





    // //waiting for the first joint position
    // while(actualState.initCheck && nextState.initCheck ){
    //     Fk_and_pub(actualState,client,pub_pos,pub_speed);
    //     ros::spinOnce();
    // } 

    // //Check orientation at the desired intial pos
    // geometry_msgs::Pose msgInitPos;
    // geometry_msgs::Twist msgInitSpeed;

    // msgInitPos.orientation.x = intialPos[0];
    // msgInitPos.orientation.y = intialPos[1];
    // msgInitPos.orientation.z = intialPos[2];
    // msgInitPos.orientation.w = intialPos[3];
    // msgInitPos.position.x    = intialPos[4];
    // msgInitPos.position.y    = intialPos[5];
    // msgInitPos.position.z    = intialPos[6];

    // pub_pos.publish(msgInitPos);
    // pub_speed.publish(msgInitSpeed);
    // // run two time to send the pos to the DS node and receive the new DS
    // ros::spinOnce(); 
    // loop_rate.sleep();  
    // ros::spinOnce();   
    // loop_rate.sleep();  
    
    // intialState={nextState.quatFromDS[0],nextState.quatFromDS[1],nextState.quatFromDS[2],nextState.quatFromDS[3],intialPos[0],intialPos[1],intialPos[2]};

    // std::cout << "Desired position: ";
    // for (const double& element : intialState) {
    //     std::cout << element << " ";
    // }
    // std::cout << std::endl;
            
    // //go to first pos     
    // int rc = nextState.getIK(actualState.posJointActual,intialState);

    // if (rc < 0){
    //     ROS_ERROR("your intial point is not achiveable");
    //     return 1;
    // }
    // string UserInput = "stop";


    // while (!mseValue_cart(actualState.posJointActual,nextState.posJointNext,tol_mse) ){
    //     Fk_and_pub(actualState,client,pub_pos,pub_speed);

    //     chatter_pub.publish(nextState.msgP);
    //     ros::spinOnce();        
    //     loop_rate.sleep();  
    // }




    // ROS_INFO("first position reached, please Press GO when ready to shotcreet");
    // while( UserInput != "GO"){

    //     Fk_and_pub(actualState,client,pub_pos,pub_speed);

    //     cin >> UserInput;
    // }
    // // rostopic pub /passive_control/vel_quat geometry_msgs/Pose '{position: {x: 0.05 ,y: 0.0, z: -0.05}, orientation: {x: 0, y: 0.9848, z: 0, w: 0.1736}}'

    // //------------------------------------------------------------------------------------------------------------------------//
    // //send first pos

    // //update publisher for DS
    // update_publisher_for_DS(actualState,actualState.posJointActual, pub_pos, pub_speed,client, loop_rate);

    // vector<double> nextPosCartTemp  = actualState.posCartActual;
    // vector<double> nextPosJointTemp = nextState.posJointNext;

    // //use the speed from topic and convert the quat from topic to angular velocity
    // VectorXd speed_eigen = speed_func(actualState.posCartActual, nextState.quatFromDS,nextState.speedFromDS);

    // //integrate the speed with the actual cartesian state to find new cartesian state. The output is in  (quat,pos)
    // vector<double> NextQuatPosCart = Integral_func(actualState.posCartActual, speed_eigen, integrationTime, nextState);
    
    // //compute inverse kinematic
    // nextState.getIK(nextPosJointTemp,NextQuatPosCart);

    // // publish to the iiwa position controller
    // std_msgs::Float64MultiArray nextPosJointTempMsg = nextState.msgP;
    // chatter_pub.publish(nextPosJointTempMsg);

    // nextPosJointTemp= nextState.posJointNext;
    // nextPosCartTemp = actualState.getFK(nextPosJointTemp);

    // // publish the next cart position for the DS
    // pub_pos.publish(actualState.actualCart); // the name is not clear but it's send a msg with the values of nextPosCartTemp

    // // we should update the twist as well

    // ros::spinOnce();        
    // loop_rate.sleep();  

    // //begin the ros loop

    // bool check_ik = true;
    // ROS_INFO("LETSGO");
    // while (ros::ok())
    // {
    //             //compute nextpos +1 
    //     VectorXd speed_eigen = speed_func(nextPosCartTemp, nextState.quatFromDS,nextState.speedFromDS);
    //     vector<double> NextQuatPosCart = Integral_func(nextPosCartTemp, speed_eigen, integrationTime, nextState);
        
    //     nextState.getIK(nextPosJointTemp,NextQuatPosCart);

    //     update_publisher_for_DS(actualState, nextState.posJointNext, pub_pos, pub_speed,client, loop_rate);


    //     nextPosJointTemp = nextState.posJointNext;
    //     nextPosCartTemp = actualState.getFK(nextPosJointTemp); 
    //     nextPosJointTempMsg = nextState.msgP;
    //     // Print the vector elements
    //     cout << "Vector elements: ";
    //     for (const double& value : NextQuatPosCart) {
    //         cout << value << " ";
    //     }
    //     cout << endl;
    //     ros::spinOnce();        
    //     loop_rate.sleep();  
    //     //--------------------------------------------------------------------
    //     //send next joint 
    //     chatter_pub.publish(nextPosJointTempMsg);
    // }
    // return 0;    

