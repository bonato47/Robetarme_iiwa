#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Float64MultiArray.h"
#include <cmath>
#include <iostream>
#include <sstream>
#include "iiwa_tools/GetFK.h"
#include "std_srvs/Empty.h"
#include <eigen3/Eigen/Dense>
#include <stdio.h>
#include <trac_ik/trac_ik.hpp>
#include "Send_pos_function.h"
#include "Utils.h"


using namespace Eigen;
using namespace std;

int n =7;
vector<double> pos_joint_actual(n);
vector<double> vel(n);
vector<double> eff(n);

void CounterCallback(const sensor_msgs::JointState::ConstPtr msg);

  // Function that Calculate the speed with a DS
VectorXd speed_func(vector<double> Pos, Vector3d x01, Vector4d q2);

// Function that integrate the speed
VectorXd Integral_func(vector<double> Pos_actual, VectorXd speed_actual, double dt);

// Function that Calculate Root Mean Square
bool mseValue_cart(vector<double> v1, vector<double> v2);

//Define work space

int main(int argc, char **argv)
{

    Vector4d Orientation_des;
    Orientation_des << 0.5,0.5,0.5,1.0 ;
    Vector3d Position_des;
    Position_des << 0.5,0.5,0.5;

    iiwa_tools::GetFK  FK_state ;

    double delta_t = 0.01;
   
    //Initialisation of the Ros Node (Service, Subscrber and Publisher)
    ros::init(argc, argv, "Ds");
    ros::NodeHandle Nh_;
    ros::ServiceClient client_FK = Nh_.serviceClient<iiwa_tools::GetFK>("iiwa/iiwa_fk_server");
    ros::Subscriber sub = Nh_.subscribe("iiwa/joint_states", 1000, CounterCallback);
    ros::Publisher chatter_pub = Nh_.advertise<std_msgs::Float64MultiArray>("iiwa/PositionController/command", 1000);
    ros::Rate loop_rate(1/0.01);//1/delta_t);

    //iniailization Forward Kinematics

    FK_state.request.joints.layout.dim.push_back(std_msgs::MultiArrayDimension());
    FK_state.request.joints.layout.dim.push_back(std_msgs::MultiArrayDimension());
    FK_state.request.joints.layout.dim[0].size = 1;
    FK_state.request.joints.layout.dim[1].size = 7;

    //iniailization Invers Kinematics
    string base_link = "iiwa_link_0";
    string tip_link = "iiwa_link_ee";
    string URDF_param="/robot_description";
    double timeout_in_secs=0.05;
    double error=1e-3; 
    TRAC_IK::SolveType type=TRAC_IK::Distance;
    TRAC_IK::TRAC_IK ik_solver(base_link, tip_link, URDF_param, timeout_in_secs, error, type);  

    KDL::Chain chain;

    bool valid = ik_solver.getKDLChain(chain);
    if (!valid)
    {
        ROS_ERROR("There was no valid KDL chain found");
    }

    //initialization  Variable
    VectorXd speed,pos_cart_N;
    std_msgs::Float64MultiArray msgP;
    std_msgs::Float64MultiArray Past_joint_pos;
    std_msgs::Float64MultiArray Next_joint_pos;
    geometry_msgs::Quaternion Past_cart_quat;
    geometry_msgs::Pose Past_cart;
    geometry_msgs::Point Past_cart_pos;
    vector<double> Pos_cart_actual(n);
    vector<double> Quat_N(4);
    vector<double> pos_joint_next(n);
    int count = 0;

    //begin the ros loop
    while (ros::ok())
    {
        // Take joints state actual and convert to cartesian state
        // With the help of th FK service
        //ROS_INFO("Actual state");

        Past_joint_pos.data = {pos_joint_actual[0],pos_joint_actual[1],pos_joint_actual[2],pos_joint_actual[3],pos_joint_actual[4],pos_joint_actual[5],pos_joint_actual[6]};
        FK_state.request.joints.data = Past_joint_pos.data;

        client_FK.call(FK_state);
        Past_cart = FK_state.response.poses[0], FK_state.response.poses[1];
        Past_cart_pos = Past_cart.position;
        Past_cart_quat = Past_cart.orientation;
        
        //ROS_INFO("%f %f %f %f %f %f %f",pos_joint_actual[0],pos_joint_actual[1],pos_joint_actual[2],pos_joint_actual[3],pos_joint_actual[4],pos_joint_actual[5],pos_joint_actual[6]);
        ROS_INFO("%f %f %f %f",Past_cart_quat.x,Past_cart_quat.y,Past_cart_quat.z,Past_cart_quat.w);
  
        Pos_cart_actual={Past_cart_quat.x,Past_cart_quat.y,Past_cart_quat.z,Past_cart_quat.w,Past_cart_pos.x,Past_cart_pos.y,Past_cart_pos.z};

        //-----------------------------------------------------------------------
        //Send the cartesian stat to Dynamical System (DS) to find desired speed

        // need to add the orientation
        //ROS_INFO("Desired speed");

        speed = speed_func(Pos_cart_actual,Position_des, Orientation_des);
        //ROS_INFO("%f %f %f", speed[0],speed[1],speed[2]);

        //-----------------------------------------------------------------------
        //integrate the speed with the actual cartesian state to find new cartesian state
        //ROS_INFO("Desired cartesian position");


        pos_cart_N = Integral_func(Pos_cart_actual, speed, delta_t);
        //ROS_INFO("%f %f %f",pos_cart_N[0],pos_cart_N[1],pos_cart_N[2]);

        //------------------------------------------------------------------------
        //Convert cartesian to joint space
        KDL::JntArray Next_joint_task;
        KDL::JntArray actual_joint_task;   

        double* ptr = &pos_joint_actual[0];
        Map<VectorXd> pos_joint_actual_eigen(ptr, 7); 
        actual_joint_task.data = pos_joint_actual_eigen;

       /*  KDL::Rotation Rot = KDL::Rotation::Quaternion(Quat_N[0],Quat_N[1],Quat_N[2],Quat_N[3]);
    
        KDL::Vector Vec(pos_cart_N[0],pos_cart_N[1],pos_cart_N[2]); */

        KDL::Rotation Rot = KDL::Rotation::EulerZYX(pos_cart_N[2],pos_cart_N[1],pos_cart_N[0]);
        KDL::Vector Vec(pos_cart_N[3],pos_cart_N[4],pos_cart_N[5]);


        KDL::Frame Next_joint_cartesian(Rot,Vec); 

        VectorXd pos_joint_next_eigen ;
        int rc = ik_solver.CartToJnt(actual_joint_task, Next_joint_cartesian, Next_joint_task);
        //ROS_INFO("%d",rc);

        pos_joint_next_eigen = Next_joint_task.data;
        for(int i = 0 ;i<7;++i){
            pos_joint_next[i] =pos_joint_next_eigen(i);
        }

        // Filter
       
        double alpha = 0.2;
        vector<double> pos_joint_next_filter(7);

        for(int i = 0;i<7;i++){
            pos_joint_next_filter[i] = alpha*pos_joint_next[i] +(1-alpha)*pos_joint_actual[i];
        }
        //-----------------------------------------------------------------------
        //send next joint and wait
        if(count > 0 && mseValue_cart({Position_des[0],Position_des[1],Position_des[2]},Pos_cart_actual)){
            msgP.data = pos_joint_next_filter;
            chatter_pub.publish(msgP);
        }
        //--------------------------------------------------------------------
        //ROS_INFO("%f %f %f %f %f %f %f ", Pos_cart_actual[0],Pos_cart_actual[1],Pos_cart_actual[2],Pos_cart_actual[3],Pos_cart_actual[4],Pos_cart_actual[5],Pos_cart_actual[6]);

        ros::spinOnce();        
        loop_rate.sleep();  
        ++count;      
    }
    return 0;
}

void CounterCallback(const sensor_msgs::JointState::ConstPtr msg)
{
    pos_joint_actual = msg->position;
    vel = msg->velocity;
    eff = msg->effort;
}

VectorXd speed_func(vector<double> Pos,Vector3d x01, Vector4d q2)
{
    //int num_gridpoints = 30;
    Vector3d Position ;
    Position[0]= Pos[4];
    Position[1]= Pos[5];
    Position[2]= Pos[6];
    Matrix3d A;
    //Set a linear DS
    A << -1, 0, 0 ,
          0,-1, 0 ,
          0, 0,-1;
    A =0.1*A;
    Vector3d b1,w; 
    // Set the attracotr
    //x01 << 0.5,0.5,0.5;
    b1 =  -A*x01;
    w = A *Position +b1 ;
    //w = w.normalized()*0.1;

     //orientation
    Vector4d q1 ;
    q1 << Pos[0],Pos[1],Pos[2],Pos[3];
    Vector4d dqd = Utils<double>::slerpQuaternion(q1, q2 ,0.5);    
    Vector4d deltaQ = dqd -  q1;

    Vector4d qconj = q1;
    qconj.segment(1,3) = -1 * qconj.segment(1,3);
    Vector4d temp_angVel = Utils<double>::quaternionProduct(deltaQ, qconj);

    Vector3d tmp_angular_vel = temp_angVel.segment(1,3);
    double maxDq = 0.2;
    if (tmp_angular_vel.norm() > maxDq)
        tmp_angular_vel = maxDq * tmp_angular_vel.normalized();

    double dsGain_ori = 2.50;
    double theta_gq = (-.5/(4*maxDq*maxDq)) * tmp_angular_vel.transpose() * tmp_angular_vel;
    Vector3d Omega_out  = 2 * dsGain_ori*(1+std::exp(theta_gq)) * tmp_angular_vel;
    

    vector<double> V = {Omega_out[0],Omega_out[1],Omega_out[2],w[0],w[1],w[2]};
    //vector<double> V = {0,0,0,w[0],w[1],w[2]};

    double* pt = &V[0];
    VectorXd VOut = Map<VectorXd>(pt, 6);

    return VOut;
}

VectorXd Integral_func(vector<double> Pos_actual, VectorXd speed_actual, double dt)
{
    Quaterniond q ={Pos_actual[0],Pos_actual[1],Pos_actual[2],Pos_actual[3]};
    Vector3d euler = q.toRotationMatrix().eulerAngles(0, 1, 2);
    VectorXd Position_cart_euler(6) ;
    Position_cart_euler[0]= euler[0];
    Position_cart_euler[1]= euler[1];
    Position_cart_euler[2]= euler[2];
    Position_cart_euler[3]= Pos_actual[3];
    Position_cart_euler[4]= Pos_actual[4];
    Position_cart_euler[5]= Pos_actual[5];


    VectorXd pos_cart_Next;
    pos_cart_Next = speed_actual * dt + Position_cart_euler;
    return pos_cart_Next;
}

// Function that Calculate Root Mean Square
bool mseValue_cart(vector<double> v1, vector<double> v2)
{
    // tolerance of the errot between each point
    float tol =0.05;
    bool Reached = false;
    int crit =0;
    float err =0;
    int Len = v1.size();

    for (int i = 0; i < Len; i++) {
        err = err + (v1[i]-v2[i])*(v1[i]-v2[i]);
    }
    if(sqrt(err) > tol){
        Reached =true;
    }

    return Reached;
}

