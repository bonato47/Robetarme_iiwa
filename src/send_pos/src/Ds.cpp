#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Float64MultiArray.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Point.h"
#include <cmath>
#include <iostream>
#include <sstream>
#include "iiwa_tools/GetFK.h"
#include "std_srvs/Empty.h"
#include <eigen3/Eigen/Dense>
#include <stdio.h>
#include <trac_ik/trac_ik.hpp>
#include "Send_pos_function.h"

using namespace Eigen;
using namespace std;

int n =7;
vector<double> pos_task_actual(n);
vector<double> vel(n);
vector<double> eff(n);

void CounterCallback(const sensor_msgs::JointState::ConstPtr msg);

 // Function that Calculate the speed with a DS
Vector3d speed_func(vector<double> Pos);

// Function that integrate the speed
Vector3d Integral_func(vector<double> Pos_actual, Vector3d speed_actual, double dt);

// Function that Calculate Root Mean Square
bool mseValue(vector<double> v1, vector<double> v2);

// Function that verify if goal is reached
bool Send_pos_func(vector<double> pos,vector<double> posDes);
 


//Define work space

int main(int argc, char **argv)
{

    iiwa_tools::GetFK  FK_state ;

    double delta_t = 0.5;
   
    //Initialisation of the Ros Node (Service, Subscrber and Publisher)
    ros::init(argc, argv, "Ds");
    ros::NodeHandle Nh_;
    ros::ServiceClient client_FK = Nh_.serviceClient<iiwa_tools::GetFK>("iiwa/iiwa_fk_server");
    ros::Subscriber sub = Nh_.subscribe("iiwa/joint_states", 1000, CounterCallback);
    ros::Publisher chatter_pub = Nh_.advertise<std_msgs::Float64MultiArray>("iiwa/PositionController/command", 1000);
    ros::Rate loop_rate(1/delta_t);


    FK_state.request.joints.layout.dim.push_back(std_msgs::MultiArrayDimension());
    FK_state.request.joints.layout.dim.push_back(std_msgs::MultiArrayDimension());
    FK_state.request.joints.layout.dim[0].size = 1;
    FK_state.request.joints.layout.dim[1].size = 7;


    Vector3d speed,speed2, Pos_N,Pos_N2;
    std_msgs::Float64MultiArray msgP;
    std_msgs::Float64MultiArray Past_joint_pos;
    std_msgs::Float64MultiArray Next_joint_pos;
    geometry_msgs::Quaternion Past_cart_quat;
    geometry_msgs::Pose Past_cart;
    geometry_msgs::Point Past_cart_pos;
    vector<double> Pos_cart_actual(n);
    vector<double> Quat_N(4);
    vector<double> Pos_task_next(n);
    vector<double> Quat_N2(4);

    while (ros::ok())
    {
        // Take joints state actual and convert to cartesian state
        // With the help of th FK service
        ROS_INFO("Actual state");

        Past_joint_pos.data = {pos_task_actual[0],pos_task_actual[1],pos_task_actual[2],pos_task_actual[3],pos_task_actual[4],pos_task_actual[5],pos_task_actual[6]};
        FK_state.request.joints.data = Past_joint_pos.data;

        client_FK.call(FK_state);
        //Past_cart = FK_state.response.poses;
        Past_cart = FK_state.response.poses[0], FK_state.response.poses[1];
        Past_cart_pos = Past_cart.position;
        Past_cart_quat = Past_cart.orientation;
        
        ROS_INFO("%f %f %f %f %f %f %f",pos_task_actual[0],pos_task_actual[1],pos_task_actual[2],pos_task_actual[3],pos_task_actual[4],pos_task_actual[5],pos_task_actual[6]);
        ROS_INFO("%f %f %f",Past_cart_pos.x, Past_cart_pos.y, Past_cart_pos.z);
  
        Pos_cart_actual={Past_cart_pos.x,Past_cart_pos.y,Past_cart_pos.z,Past_cart_quat.x,Past_cart_quat.y,Past_cart_quat.z,Past_cart_quat.w};

        //-----------------------------------------------------------------------
        //Send the cartesian stat to Dynamical System (DS) to find desired speed

        // need to add the orientation
        ROS_INFO("Desired speed");

        speed = speed_func(Pos_cart_actual);
        ROS_INFO("%f %f %f",speed[0],speed[1],speed[2]);

        //-----------------------------------------------------------------------
        //integrate the speed with the actual cartesian state to find new cartesian state
        ROS_INFO("Desired cartesian position");
        Pos_N = Integral_func(Pos_cart_actual, speed, delta_t);
        ROS_INFO("%f %f %f",Pos_N[0],Pos_N[1],Pos_N[2]);

        // need to add the orientation
        Quat_N = {0,0,0.7,-0.7};
       
        
        vector<double> Temp ={Pos_N[0],Pos_N[1],Pos_N[2],Quat_N[0],Quat_N[1],Quat_N[2],Quat_N[3]};
/*
       //------------------------------------------------------------------------
        //Nextnext point
        ROS_INFO("Desired cartesian position 2 ");

        speed2 = speed_func(Temp);
        
        Pos_N2 = Integral_func(Temp, speed2, delta_t);
        Quat_N2 = {0,0,0.7,-0.7};
        
        ROS_INFO("%f %f %f",Pos_N2[0],Pos_N2[1],Pos_N2[2]);


        //-----------------------------------------------------------------------
        //convert cartesian state to joint state with IK, with two position

        
        vector<vector<double>> traj_cart ={{Pos_N[0],Pos_N[1],Pos_N[2],Quat_N[0],Quat_N[1],Quat_N[2],Quat_N[3]},
                                            {Pos_N2[0],Pos_N2[1],Pos_N2[2],Quat_N2[0],Quat_N2[1],Quat_N2[2],Quat_N2[3]}};
        int Len_vec =traj_cart.size();

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
        Past_joint_pos.data = {pos_task_actual[0],pos_task_actual[1],pos_task_actual[2],pos_task_actual[3],pos_task_actual[4],pos_task_actual[5],pos_task_actual[6]};
        IK_state.request.poses = h;
        IK_state.request.seed_angles = Past_joint_pos;

        client_IK.call(IK_state);  
        
        Next_joint_pos = IK_state.response.joints;
        for(int j = 0 ;j<7;++j){
            Pos_task_next[j] = IK_state.response.joints.data[j];
        }
        ------------------------------------------------------------------------
        */
        string base_link = "iiwa_link_0";
        string tip_link = "iiwa_link_ee";
        string URDF_param="/robot_description";
        double timeout_in_secs=0.005;
        double error=1e-5;
        TRAC_IK::SolveType type=TRAC_IK::Distance;
        TRAC_IK::TRAC_IK ik_solver(base_link, tip_link, URDF_param, timeout_in_secs, error, type);  

        KDL::JntArray Next_joint_task;
        KDL::JntArray actual_joint_task;   

        double* ptr = &pos_task_actual[0];
        Map<VectorXd> pos_task_actual_eigen(ptr, 7); 
        actual_joint_task.data = pos_task_actual_eigen;

        KDL::Rotation Rot = KDL::Rotation::Quaternion(Quat_N[0],Quat_N[1],Quat_N[2],Quat_N[3]);
    
        KDL::Vector Vec(Pos_N[0],Pos_N[1],Pos_N[2]);
        KDL::Frame Next_joint_cartesian(Rot,Vec); 

        //Next_joint_cartesian.orientation = 


        int rc = ik_solver.CartToJnt(actual_joint_task, Next_joint_cartesian, &Next_joint_task);

        ROS_INFO("Next joint state:");
        ROS_INFO("%f %f %f %f %f %f %f",Pos_task_next[0],Pos_task_next[1],Pos_task_next[2],Pos_task_next[3],Pos_task_next[4],Pos_task_next[5],Pos_task_next[6]);
        //ROS_INFO("Send :");
        msgP.data = Pos_task_next;

        //-----------------------------------------------------------------------
        // Send the joint state to a service that is connected to send_pos
        //when it is done, take new point
        chatter_pub.publish(msgP);
        //bool B;
        //Send_pos_func(pos_task_actual, Pos_task_next);

        //ROS_INFO("target reached, go next one ");


        //--------------------------------------------------------------------
        ros::spinOnce();        
        loop_rate.sleep();        
    }
    return 0;
}

void CounterCallback(const sensor_msgs::JointState::ConstPtr msg)
{
    pos_task_actual = msg->position;
    vel = msg->velocity;
    eff = msg->effort;
}

Vector3d speed_func(vector<double> Pos)
{
    //int num_gridpoints = 30;
    Vector3d Position ;
    Position[0]= Pos[0];
    Position[1]= Pos[1];
    Position[2]= Pos[2];
    Matrix3d A;
    //Set a linear DS
    A << -0.1, 0, 0 ,0,-0.1,0,0,0,-0.1;
    Vector3d x01,b1,w; 
    // Set the attracotr
    x01 << 0.5,0.5,0.5;
    b1 =  -A*x01;
    w = A *Position +b1 ;
    //w = w.normalized()*0.1;
    return w;
}

Vector3d Integral_func(vector<double> Pos_actual, Vector3d speed_actual, double dt)
{
    Vector3d Position ;
    Position[0]= Pos_actual[0];
    Position[1]= Pos_actual[1];
    Position[2]= Pos_actual[2];
    Vector3d Pos_Next;
    Pos_Next = speed_actual * dt + Position;
    return Pos_Next;
}

// Function that Calculate Root Mean Square
bool mseValue(vector<double> v1, vector<double> v2)
{
    // tolerance of the errot between each point
    float tol =0.1;
    bool Reached = false;
    int crit =0;
    float err =0;

    for (int i = 0; i < 7; i++) {
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
bool Send_pos_func(vector<double> pos,vector<double> posDes)
{
    int count = 0 ;
    while (count < 50)
    {
        if (mseValue(pos,posDes) || (count >50)){
            return 0;
        }   
        ++count;
    }
    return 0;

}