#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Float64MultiArray.h"
#include <eigen3/Eigen/Dense>
#include <cmath>
#include <iostream>
#include <sstream>
#include "iiwa_tools/GetFK.h"
#include "iiwa_tools/GetJacobian.h"
#include "std_srvs/Empty.h"
#include <stdio.h>
#include "thirdparty/Utils.h"

//#include "Send_pos_function.h"

using namespace Eigen;
using namespace std;

void CounterCallback(const sensor_msgs::JointState::ConstPtr msg);

  // Function that Calculate the speed with a DS
vector<double> speed_func(vector<double> pos, Vector4d q01,Vector3d x01);

// Function that integrate the speed
vector<double> Integral_func_V2(VectorXd Pos_f, VectorXd speed_f, double dt);

// Function that Calculate Root Mean Square
bool mseValue_cart(vector<double> v1, vector<double> v2);

class State_robot {       // The class
  public:             // Access specifier
    vector<double> cart;
    vector<double> joint ;
    vector<double> cart_next;
    vector<double> joint_next ;

    VectorXd cart_eigen ;
    VectorXd joint_eigen ;
    VectorXd joint_next_eigen ;
    VectorXd cart_next_eigen ;


    std_msgs::Float64MultiArray joint_std64;

    void State_robot_actual(vector<double> V) {  // Method/function defined inside the class
        joint = V;
        int L = V.size();
        double* pt = &V[0];
        joint_eigen = Map<VectorXd>(pt, L);
        joint_std64.data = {joint[0],joint[1],joint[2],joint[3],joint[4],joint[5],joint[6]};
    }

    void State_robot_next(vector<double> V) {  // Method/function defined inside the class
        joint_next = V;
        int L = V.size();
        double* pt = &V[0];
        joint_next_eigen = Map<VectorXd>(pt, L);
        //joint_std64.data = {joint[0],joint[1],joint[2],joint[3],joint[4],joint[5],joint[6]};
    }
    void State_robot_next_cart(vector<double> V) {  // Method/function defined inside the class
        cart_next = V;
        int L = V.size();
        double* pt = &V[0];
        cart_next_eigen = Map<VectorXd>(pt, L);
        //joint_std64.data = {joint[0],joint[1],joint[2],joint[3],joint[4],joint[5],joint[6]};
    }
};

//Define work space
int n =7;
vector<double> pos_joint_actual(n);
vector<double> vel_joint_actual(n);
vector<double> eff(n);

int main(int argc, char **argv)
{
    Vector4d Orientation_des;
    Orientation_des << -0.091691 ,0.823997 ,0.460200 ,0.317549 ;
    Vector3d Position_des;
    Position_des << 0.5,0.5,0.5;

    iiwa_tools::GetFK  FK_state ;
    iiwa_tools::GetJacobian  Jac_state ;

    double delta_t = 0.01;
   
    //Initialisation of the Ros Node (Service, subscriber and publisher)
    ros::init(argc, argv, "Ds_Tikhonov");
    ros::NodeHandle Nh_;
    ros::ServiceClient client_J = Nh_.serviceClient<iiwa_tools::GetJacobian>("iiwa/iiwa_jacobian_server");
    ros::ServiceClient client_FK = Nh_.serviceClient<iiwa_tools::GetFK>("iiwa/iiwa_fk_server");
    ros::Subscriber sub = Nh_.subscribe("iiwa/joint_states", 1000, CounterCallback);
    ros::Publisher chatter_pub = Nh_.advertise<std_msgs::Float64MultiArray>("iiwa/PositionController/command", 1000);
    ros::Rate loop_rate(1/delta_t);

    //initalization Forward Kinematics
    FK_state.request.joints.layout.dim.push_back(std_msgs::MultiArrayDimension());
    FK_state.request.joints.layout.dim.push_back(std_msgs::MultiArrayDimension());
    FK_state.request.joints.layout.dim[0].size = 1;
    FK_state.request.joints.layout.dim[1].size = 7;
   
    //initialization  Variable
    std_msgs::Float64MultiArray msgP;
    geometry_msgs::Quaternion Past_cart_quat;
    geometry_msgs::Pose Past_cart;
    geometry_msgs::Point Past_cart_pos;

    //begin the ros loop
    int count = 0;
    while (ros::ok())
    {
        //define object Position and speed
        State_robot Robot_position;
        Robot_position.State_robot_actual(pos_joint_actual);

        State_robot Robot_speed;
        Robot_speed.State_robot_actual(vel_joint_actual);
 
        // Take joints state actual and convert to cartesian state with the help of th FK service
        FK_state.request.joints.data = Robot_position.joint_std64.data;
        client_FK.call(FK_state);
        Past_cart = FK_state.response.poses[0], FK_state.response.poses[1];
        Past_cart_quat = Past_cart.orientation;  
        Past_cart_pos = Past_cart.position;
        Robot_position.cart = {Past_cart_quat.x,Past_cart_quat.y,Past_cart_quat.z,Past_cart_quat.w,Past_cart_pos.x,Past_cart_pos.y,Past_cart_pos.z};

        //-----------------------------------------------------------------------
        //Find the desired speed ( omega_dot ,x_dot) size 6x1
        
        //Send the cartesian stat to Dynamical System (DS) to find x_dot
        // use quatertion to speed algoritm to omega_dot
        Robot_speed.cart_next = speed_func(Robot_position.cart,Orientation_des,Position_des);
        Robot_speed.State_robot_next_cart(Robot_speed.cart_next);

        //ROS_INFO("%f %f %f ", Robot_speed.cart_next_eigen[3],Robot_speed.cart_next_eigen[4],Robot_speed.cart_next_eigen[5]);

        // use Tkihonov optimization norm(J*q_dot-V)²+ norm(w*I*q_dot)²
        //q_dot = inv(J_transpose*J+ W_transpose*W)*J_transpose*Speed

        Jac_state.request.joint_angles = Robot_position.joint_std64.data;
        Jac_state.request.joint_velocities =  Robot_speed.joint_std64.data;;
        client_J.call(Jac_state);

        std_msgs::Float64MultiArray Jacobian = Jac_state.response.jacobian;
        double* ptr = &Jacobian.data[0];    
        Map<MatrixXd> Eigen_Jac(ptr, 6, 7);

       /*  for(int  j= 0;j<3;j++){     
            for(int i = 0;i<7;i++){  
                Eigen_Jac(j,i)= 0;
            }
        } */      

        double W = 2;
        MatrixXd eigen_Weight(7,7);
        eigen_Weight << W,0,0,0,0,0,0,
                        0,W,0,0,0,0,0,
                        0,0,W,0,0,0,0,
                        0,0,0,W,0,0,0,
                        0,0,0,0,W,0,0,
                        0,0,0,0,0,W,0,
                        0,0,0,0,0,0,W;

        Robot_speed.joint_next_eigen =  (Eigen_Jac.transpose()*Eigen_Jac +eigen_Weight.transpose()*eigen_Weight).inverse()*Eigen_Jac.transpose()*Robot_speed.cart_next_eigen;
        ROS_INFO("%f %f %f %f %f %f ",Robot_speed.joint_next_eigen[0],Robot_speed.joint_next_eigen[1],Robot_speed.joint_next_eigen[2],Robot_speed.joint_next_eigen[4],Robot_speed.joint_next_eigen[5],Robot_speed.joint_next_eigen[6]);


  /*       for(int i = 0;i<6*7;i++){
            ROS_INFO("%f", Jacobian.data[i]);
        } */
        Robot_position.joint_next =  Integral_func_V2(Robot_position.joint_eigen, Robot_speed.joint_next_eigen, delta_t);
        //ROS_INFO("%f %f %f",Robot_position.joint_next[0],Robot_position.joint_next[1],Robot_position.joint_next[2]);
        //ROS_INFO("%f %f %f %f %f %f ",Position_des[0],Position_des[1],Position_des[2],Robot_position.cart[4],Robot_position.cart[5],Robot_position.cart[6]);
       /*

        // Filter
       
        double alpha = 0.2;
         vector<double> pos_joint_next_filter(7);

        for(int i = 0;i<7;i++){
            pos_joint_next_filter[i] = alpha*pos_joint_next[i] +(1-alpha)*Robot_position.joint[i];
        }
         */
        //-----------------------------------------------------------------------
        //send next joint and wait
        if(count > 0 && mseValue_cart({Position_des[0],Position_des[1],Position_des[2]},{Robot_position.cart[4],Robot_position.cart[5],Robot_position.cart[6]})){
            msgP.data = Robot_position.joint_next;
            chatter_pub.publish(msgP);
        }

        //--------------------------------------------------------------------
        ros::spinOnce();        
        loop_rate.sleep();  
        ++count;      
    }
    return 0;
}

void CounterCallback(const sensor_msgs::JointState::ConstPtr msg)
{
    pos_joint_actual = msg->position;
    vel_joint_actual = msg->velocity;
    eff = msg->effort;
}

vector<double> speed_func(vector<double> pos, Vector4d q2 ,Vector3d x01)
{   
    //orientation
    Vector4d q1 ;
    q1 << pos[0],pos[1],pos[2],pos[3];
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

    //position
    Matrix3d A;
    Vector3d b1,w,cart_actual; 
    cart_actual << pos[4],pos[5],pos[6];

    //Set a linear DS
    A << -1, 0, 0,
          0,-1, 0,
          0, 0,-1;
    // Set the Position_des
    b1 = -A * x01;
    w  =  A * cart_actual + b1 ;

    //out
    vector<double> Vout = {Omega_out[0],Omega_out[1],Omega_out[2],w[0],w[1],w[2]};
    return Vout;
}

vector<double> Integral_func_V2(VectorXd Pos_f, VectorXd speed_f, double dt)
{
    VectorXd pos_Next_eigen_f;
    pos_Next_eigen_f = speed_f * dt + Pos_f;
    vector<double> pos_Next_f= {pos_Next_eigen_f[0],pos_Next_eigen_f[1],pos_Next_eigen_f[2],pos_Next_eigen_f[3],pos_Next_eigen_f[4],pos_Next_eigen_f[5],pos_Next_eigen_f[6]};
    return pos_Next_f;
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


