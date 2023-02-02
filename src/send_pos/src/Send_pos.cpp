#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Float64MultiArray.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Point.h"
#include <cmath>
#include <iostream>
#include <sstream>
#include "iiwa_tools/GetIK.h"
#include "std_srvs/Empty.h"
#include <eigen3/Eigen/Dense>

using namespace std;
using namespace Eigen;

void CounterCallback(const sensor_msgs::JointState::ConstPtr);
bool mseValue(vector<double> , vector<double> ,int);
vector<double> TakeLine(vector<vector<double>> , int );
int n =7;
vector<double> eff(n);
vector<double> vel(n);
vector<double> pos(n);
float pi =3.14;
int main(int argc, char **argv)
{
    std_msgs::Float64MultiArray msgP;
    std_msgs::Float64MultiArray J;
    std_msgs::Float64MultiArray Past_joint_pos;

    iiwa_tools::GetIK  IK_state ;
    vector<double> posDes(n);
    vector<double> V_temp(n);
    vector<vector<double>> traj_joint;
    //Choose the position of the end effector {pos,quat}
    vector<vector<double>> traj_cart{{0.004663, 0.004663 ,1.298483,0,0,0.7,-0.7},                       
                                    {0,0,1.3,0,0,0.7,-0.7}};
    int Len_vec =traj_cart.size();

    //Initialisation of the Ros Node (Service, Subscrber and Publisher)
    ros::init(argc, argv, "Send_pos");
    ros::NodeHandle Nh;
    ros::ServiceClient client = Nh.serviceClient<iiwa_tools::GetIK>("iiwa/iiwa_ik_server");
    ros::Subscriber sub = Nh.subscribe("iiwa/joint_states", 1000, CounterCallback);
    ros::Publisher chatter_pub = Nh.advertise<std_msgs::Float64MultiArray>("iiwa/PositionController/command", 1000);
    //Frequency of the Ros loop
    ros::Rate loop_rate(10);

    vector<geometry_msgs::Pose> h(Len_vec);
    for(int i = 0 ;i<Len_vec;++i){
        
        geometry_msgs::Point Pos_init ;
        Pos_init.x = traj_cart[i][0];
        Pos_init.y = traj_cart[i][1];
        Pos_init.z = traj_cart[i][2];
        geometry_msgs::Quaternion Quat_init ;
        Quat_init.x = traj_cart[i][3];
        Quat_init.y = traj_cart[i][4];
        Quat_init.z = traj_cart[i][5];
        Quat_init.w = traj_cart[i][6];

        geometry_msgs::Pose poses_init ;
        poses_init.position = Pos_init;
        poses_init.orientation = Quat_init;

        h[i] = (poses_init);
    }

    Past_joint_pos.data = {pos[0],pos[1],pos[2],pos[3],pos[4],pos[5],pos[6]};
    IK_state.request.poses = h;
    IK_state.request.seed_angles = Past_joint_pos;
    //Get IK to convert cartesian position to joint position

    client.call(IK_state);
  
    int Next  = 0;
    for(int i = 0 ;i<Len_vec;++i){
        for(int j = 0 ;j<7;++j){
            V_temp[j] = IK_state.response.joints.data[Next];
            ++Next;
        }
        traj_joint.push_back(V_temp);
    }
    ROS_INFO("Let'move on");
  
    //begin the Ros loop
    Next  = 0;
    int count = 0 ;
    while (ros::ok())
    {
        msgP.data = traj_joint[Next];
        posDes= traj_joint[Next];
        
        //ROS_INFO("%f", mseValue(pos,posDes,n));
        if ((mseValue(pos,posDes,n) && (Next < Len_vec))  || (count >50)){
            ++Next;
            if(Next == Len_vec){
                ROS_INFO("Last target reached, stop ");
                return 0; 
            }
            ROS_INFO("target reached, go next one ");
            count = 0;
        }   
        ++count;
        chatter_pub.publish(msgP);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}


void CounterCallback(const sensor_msgs::JointState::ConstPtr msg)
{
    pos = msg->position;
    vel = msg->velocity;
    eff = msg->effort;
}

// Function that Calculate Root Mean Square
bool mseValue(vector<double> v1, vector<double> v2,int Num)
{
    // tolerance of the errot between each point
    float tol =0.1;
    bool Reached = false;
    int crit =0;
    float err =0;

    for (int i = 0; i < Num; i++) {
        err = sqrt((v1[i]-v2[i])*(v1[i]-v2[i]));
        if(err > tol){
            ++crit;
        }
       // ROS_INFO("%f",err);

    }
    if(crit == 0){
        Reached =true;
    }

    return Reached;
}

vector<double> TakeLine(vector<vector<double>> Mat, int numB )
{
    vector<double> vector(7);
    for (int i = 0; i < 7; i++) {
        vector[i] = Mat[numB][i];
    }

    return vector;
}