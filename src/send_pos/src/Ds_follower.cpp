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

void CounterCallback(const sensor_msgs::JointState::ConstPtr msg);

  // Function that Calculate the speed with a DS
VectorXd speed_func(vector<double> Pos, Vector3d x01, Vector4d q2);

// Function that integrate the speed
VectorXd Integral_func(vector<double> Pos_actual, VectorXd speed_actual, double dt);

// Function that Calculate Root Mean Square
bool mseValue_cart(vector<double> v1, vector<double> v2);

//vector<double> DS_basic(class State_robot &Position, class State_robot &Speed, double dt);


class ActualState {       // The class
  public:             // Access specifier
    int nJoint =7;
    bool initCheck= false;
    vector<double> cart,joint;
    VectorXd cart_eigen, joint_eigen;
    std_msgs::Float64MultiArray joint_std64;

    geometry_msgs::Quaternion Past_cart_quat;
    geometry_msgs::Pose Past_cart;
    geometry_msgs::Point Past_cart_pos;


    vector<double> posJointActual;
    vector<double> posCartActual;

    VectorXd posJointActualEigen;
    VectorXd posCartActualEigen;
    ros::ServiceClient client_FK;

    iiwa_tools::GetFK  FK_state ;

    void init(ros::ServiceClient FK){
        client_FK = FK;
        vector<double> vector0(nJoint, 0.0);
        posJointActual = vector0;
        posCartActual  = vector0;
        double* pt = &posJointActual[0];
        joint_eigen = Map<VectorXd>(pt, 7);
        joint_std64.data = {posJointActual[0],posJointActual[1],posJointActual[2],posJointActual[3],posJointActual[4],posJointActual[5],posJointActual[6]};
        initFK();
        }
    void initFK(){
         //iniailization Forward Kinematics
        FK_state.request.joints.layout.dim.push_back(std_msgs::MultiArrayDimension());
        FK_state.request.joints.layout.dim.push_back(std_msgs::MultiArrayDimension());
        FK_state.request.joints.layout.dim[0].size = 1;
        FK_state.request.joints.layout.dim[1].size = nJoint;
    }
     void getFK(){
        // Take joints state actual and convert to cartesian state with the help of th FK service
        FK_state.request.joints.data =  joint_std64.data;
        client_FK.call(FK_state);
        Past_cart = FK_state.response.poses[0], FK_state.response.poses[1];
        Past_cart_pos = Past_cart.position;
        Past_cart_quat = Past_cart.orientation;
        posCartActual = {Past_cart_quat.x,Past_cart_quat.y,Past_cart_quat.z,Past_cart_quat.w,Past_cart_pos.x,Past_cart_pos.y,Past_cart_pos.z};
    }

    void CounterCallback(const sensor_msgs::JointState::ConstPtr msg)
    {
        posJointActual = msg->position;
        posJointActualEigen = Map<VectorXd>(&posJointActual[0], nJoint);

        if(initCheck == false){
            initCheck = true;
        }    
    }
};

class NextState {       // The class
    public:             // Access specifier
        //iniailization Invers Kinematics
        string baseLink;
        string tipLink;
        string URDF_param="/robot_description";
        TRAC_IK::SolveType type =TRAC_IK::Distance;
        double error=1e-3; 
        double timeoutInSecs=0.05;
        int nJoint{};

        vector<double> posJointActual;
        vector<double> velJointActual;
        vector<double> posJointNext;
        VectorXd velJointActualEigen;
        VectorXd posJointActualEigen;

        vector<double> posJointNext;
        VectorXd posJointNextEigen;

        vector<double> speedFromDS;
        vector<double> quatFromDS;



        ros::V_string jointsName;
        bool init_check= false;
        TRAC_IK::TRAC_IK* ikSolver = nullptr;  


    void init_IK_iiwa() {  // Method/function defined inside the class
        baseLink  = "iiwa_link_0";
        tipLink   = "iiwa_link_ee";
        nJoint    = 7;
        init_general();
    }    
    
    void init_general(){
        vector<double> vector0(4, 0.0);;
        quatFromDS = vector0;
        vector<double> vector0(3, 0.0);;
        SpeedFromDS = vector0;

        ikSolver= new TRAC_IK::TRAC_IK(baseLink, tipLink, URDF_param, timeoutInSecs, error, type);  
        KDL::Chain chain;
        bool valid = ikSolver->getKDLChain(chain);
        if (!valid) {
        ROS_ERROR("There was no valid KDL chain found");
        } 
    }

    int getIK(vector<double> vectorQuatPos ) {  
        //Inverse kinematics trac-IK
        KDL::JntArray NextJointTask;
        KDL::JntArray actualJointTask; 
        actualJointTask.data = posJointNextEigen; 
        std::fill(posJointNext.begin(), posJointNext.end(), 0);
        KDL::Vector Vec(vectorQuatPos[4],vectorQuatPos[5],vectorQuatPos[6]);
        KDL::Rotation Rot = KDL::Rotation::Quaternion(vectorQuatPos[0],vectorQuatPos[1],vectorQuatPos[2],vectorQuatPos[3]);
        KDL::Frame NextJointCartesian(Rot,Vec); 
        int rc = ikSolver->CartToJnt(actualJointTask, NextJointCartesian, NextJointTask);

        posJointNextEigen = NextJointTask.data;
        for(int i = 0 ;i<nJoint;++i){
            posJointNext[i] =posJointNextEigen(i);
        }
        return rc;
     } 

     void updateIK(double err){
        ikSolver= new TRAC_IK::TRAC_IK(baseLink, tipLink, URDF_param, timeoutInSecs, err, type);  
     }

    void poseCallback(const geometry_msgs::Pose::ConstPtr& msg) {
        // Process the received Pose message here
        // For example, you can access the position and orientation as follows:
        double x = msg->position.x;
        double y = msg->position.y;
        double z = msg->position.z;

        double qx = msg->orientation.x;
        double qy = msg->orientation.y;
        double qz = msg->orientation.z;
        double qw = msg->orientation.w;

        speedFromDS = [x,y,z]
        quatFromDS= [qx,qy,qz,qw];   
    }

};

int main(int argc, char **argv)
{

    //choose the time step
    double delta_t = 0.01;

    //Initialisation of the Ros Node (Service, Subscrber and Publisher)
    ros::init(argc, argv, "Ds");
    ros::NodeHandle Nh_;
    ros::ServiceClient FK = Nh_.serviceClient<iiwa_tools::GetFK>("iiwa/iiwa_fk_server");
    ros::Publisher chatter_pub = Nh_.advertise<std_msgs::Float64MultiArray>("iiwa/PositionController/command", 1000);
    ros::Rate loop_rate(1/delta_t);

    //Define object position and speed
    ActualState actualState;
    actualState.init(FK);
    NextState nextstate;
    nextState.init_IK_iiwa();

    ros::Subscriber sub =  Nh_.subscribe("iiwa/joint_states", 1000, &ActualState::CounterCallback, &actualState);
    ros::Subscriber sub_DS =  Nh_.subscribe("/passive_control/vel_quat", 1000, &NextState::poseCallback, &nextState);


    int count = 0;

    //begin the ros loop
    while (ros::ok())
    {

        //waiting for the first joint position
        while(!actualState.initCheck){
            ros::spinOnce();
        } 
      
        //FK
        actualState.getFK();
        ROS_INFO("%f",actualState.posCartActual[0]);
       




//         //-----------------------------------------------------------------------
//         //Send the cartesian stat to Dynamical System (DS) to find desired speed ( wx,wy,wz,px,py,pz)
//          VectorXd speed_cart;
//         Robot_speed.cart_next_eigen = speed_func(Robot_position.cart,Position_des, Orientation_des);
//         //-----------------------------------------------------------------------

//         //integrate the speed with the actual cartesian state to find new cartesian state. The output is in  (quat,pos)
//         Robot_position.cart_next_eigen = Integral_func(Robot_position.cart, Robot_speed.cart_next_eigen, delta_t);
        
//         //------------------------------------------------------------------------
//         //Convert cartesian to joint space
//         KDL::JntArray Next_joint_task;
//         KDL::JntArray actual_joint_task;   
//         actual_joint_task.data = Robot_position.joint_eigen;

//         KDL::Rotation Rot = KDL::Rotation::Quaternion(Robot_position.cart_next_eigen[0],Robot_position.cart_next_eigen[1],Robot_position.cart_next_eigen[2],Robot_position.cart_next_eigen[3]);
//         KDL::Vector Vec(Robot_position.cart_next_eigen[4],Robot_position.cart_next_eigen[5],Robot_position.cart_next_eigen[6]);
//         KDL::Frame Next_joint_cartesian(Rot,Vec); 

//         int rc = ik_solver.CartToJnt(actual_joint_task, Next_joint_cartesian, Next_joint_task);

//         vector<double> pos_joint_next(7);
//         Robot_position.joint_next_eigen = Next_joint_task.data;
//         for(int i = 0 ;i<7;++i){
//             pos_joint_next[i] = Robot_position.joint_next_eigen(i);
//         }
//         Robot_position.joint_next = pos_joint_next;
//         //-----------------------------------------------------------------------
//         // Filter
       
// /*         double alpha = 0.2;
//         vector<double> pos_joint_next_filter(7);

//         for(int i = 0;i<7;i++){
//             pos_joint_next_filter[i] = alpha*pos_joint_next[i] +(1-alpha)*pos_joint_actual[i];
//         } */
//         //-----------------------------------------------------------------------
//         //send next joint and exit if arrived to the attractor
//         if(count > 0 && mseValue_cart({Position_des[0],Position_des[1],Position_des[2]},{Robot_position.cart[4],Robot_position.cart[5],Robot_position.cart[6]})){
//             msgP.data = Robot_position.joint_next;
//             chatter_pub.publish(msgP);
//         }
//         else{
//             if(count > 0){
//                 ROS_INFO(" Attractor Reached, Exit");
//                 return 0;
//             }
//         }
//         //--------------------------------------------------------------------
//         ros::spinOnce();        
//         loop_rate.sleep();  
//         ++count;      
    }
    return 0;
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
    A =50*A;
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

    double dsGain_ori = 0.50;
    double theta_gq = (-.5/(4*maxDq*maxDq)) * tmp_angular_vel.transpose() * tmp_angular_vel;
    Vector3d Omega_out  = 2 * dsGain_ori*(1+std::exp(theta_gq)) * tmp_angular_vel;
    
    vector<double> V = {Omega_out[0],Omega_out[1],Omega_out[2],w[0],w[1],w[2]};

    double* pt = &V[0];
    VectorXd VOut = Map<VectorXd>(pt, 6);

    return VOut;
}

VectorXd Integral_func(vector<double> Pos_actual, VectorXd speed_actual, double dt)
{
    //Speed orientation integration
    
    Quaterniond q2 ={Pos_actual[0],Pos_actual[1],Pos_actual[2],Pos_actual[3]};
    //Vector3d euler = q.toRotationMatrix().eulerAngles(0, 1, 2);
    Vector3d Speedbis ={dt*speed_actual[0]/2,dt*speed_actual[1]/2,dt*speed_actual[2]/2};
    Quaterniond q1 ={Speedbis[0],Speedbis[1],Speedbis[2],1};

    Quaterniond resultQ;
    resultQ.setIdentity();

    resultQ.w() = q1.w() * q2.w() - q1.vec().dot(q2.vec());
    resultQ.vec() = q1.w() * q2.vec() + q2.w() * q1.vec() + q1.vec().cross(q2.vec());
    resultQ.normalize();
    resultQ.w() = q2.w() + resultQ.w();
    resultQ.vec() = q2.vec() + resultQ.vec();
    
    ////ROS_INFO(" %f %f %f %f ", q2.x(),q2.y(),q2.z(),q2.w());
    //ROS_INFO(" %f %f %f %f ", resultQ.x(),resultQ.y(),resultQ.z(),resultQ.w());
  
    //Speed position integration
    Vector3d speed_bis,past_bis,next_bis;
    speed_bis << speed_actual[3],speed_actual[4],speed_actual[5];
    past_bis << Pos_actual[4],Pos_actual[5],Pos_actual[6];
    next_bis =speed_bis*dt+ past_bis ;

    VectorXd pos_cart_Next(7);

    pos_cart_Next<< resultQ.x(),resultQ.y(),resultQ.z(),resultQ.w(),next_bis;
    //pos_cart_Next<< 0,0,0,1,next_bis;

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
/* 
vector<double> DS_basic(class State_robot &Position, class State_robot &Speed, double dt,TRAC_IK::TRAC_IK &ik){
    //integrate the speed with the actual cartesian state to find new cartesian state. The output is in  (quat,pos)

    Position.cart_next_eigen = Integral_func(Position.cart, Speed.cart_eigen, dt);
    //------------------------------------------------------------------------
    //Convert cartesian to joint space
    KDL::JntArray Next_joint_task;
    KDL::JntArray actual_joint_task;   

    double* ptr = &pos_joint_actual[0];
    Map<VectorXd> pos_joint_actual_eigen(ptr, 7); 
    actual_joint_task.data = pos_joint_actual_eigen;

    //Position.joint_std64

    KDL::Rotation Rot = KDL::Rotation::Quaternion(Position.cart_next[0],Position.cart_next[1],Position.cart_next[2],Position.cart_next[3]);
    KDL::Vector Vec(Position.cart_next[4],Position.cart_next[5],Position.cart_next[6]);
    KDL::Frame Next_joint_cartesian(Rot,Vec); 

    VectorXd pos_joint_next_eigen ;
    int rc = ik.CartToJnt(Position.joint_std64, Next_joint_cartesian, Next_joint_task);

    Position.Next_joint_eigen = Next_joint_task.data;
    for(int i = 0 ;i<7;++i){
        Position.Next_joint[i] =Position.joint_eigen(i);
    }
    return Position.Next_joint;

} */

