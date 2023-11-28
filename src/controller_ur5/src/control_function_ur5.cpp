
#include "control_function_ur5.h"
#include <cmath>
#include <iostream>
#include <sstream>
#include <eigen3/Eigen/Dense>
#include <trac_ik/trac_ik.hpp>
#include "dynamical_systems/DynamicalSystemFactory.hpp"
#include "state_representation/space/cartesian/CartesianPose.hpp"
#include "state_representation/space/cartesian/CartesianTwist.hpp"
#include "state_representation/space/joint/JointPositions.hpp"
#include "state_representation/space/joint/JointVelocities.hpp"
#include <vector>
#include "Utils.h"

using namespace std;
using namespace Eigen;

RobotParameter::RobotParameter(){
    robot_name = "ur5_robot";
    tipLink  = "wrist_3_link";
    tipJoint = "wrist_3_joint";

    reference_frame = "world";
    path_urdf = "/home/ros/ros_ws/src/controller_ur5/urdf/ur5.urdf";
    nJoint    = 6;

    //parameter for inverse velocities
    alphaVel = 0.1;
    proportional_gain = 1.0;
    linear_velocity_limit = 2.0;
    angular_velocity_limit = 2.0;  
    chrono::nanoseconds t = 1000ns;
    paramsVel = {alphaVel,proportional_gain,linear_velocity_limit,angular_velocity_limit,t};

    model = make_unique<robot_model::Model>(robot_name, path_urdf);
}


vector<double> RobotParameter::getFK(vector<double> vectJoint){
    Map<VectorXd> posJoint_eigen(vectJoint.data(), vectJoint.size());
    state_representation::JointPositions nextJoinState =  state_representation::JointPositions(robot_name,posJoint_eigen);        
    state_representation::CartesianPose nextCartesianPose = model->forward_kinematics(nextJoinState,tipLink);

    Vector3d p1Prime = nextCartesianPose.get_position();
    Quaterniond q1Prime = nextCartesianPose.get_orientation();

    vector<double> posCart ={ q1Prime.x(),q1Prime.y(),q1Prime.z(),q1Prime.w(),p1Prime[0],p1Prime[1],p1Prime[2]};
    return posCart;
}
geometry_msgs::Twist RobotParameter::getTwist(vector<double> posJoint, vector<double> speedJoint){

    // Create a Twist message
    geometry_msgs::Twist twist;

    MatrixXd jacMatrix = getJacobian(posJoint);

    VectorXd speedJointEigen(nJoint);
    for(int i = 0 ;i<nJoint;++i){
        speedJointEigen(i) =speedJoint[i];
    }

    VectorXd result = jacMatrix * speedJointEigen;

    // Populate linear velocities
    twist.linear.x = result(0);
    twist.linear.y = result(1);
    twist.linear.z = result(2);

    // Populate angular velocities
    twist.angular.x = result(3);
    twist.angular.y = result(4);
    twist.angular.z = result(5); // Angular velocity around the z-axis

    return twist;
}

MatrixXd RobotParameter::getJacobian(vector<double> vectJoint){
    Map<VectorXd> posJoint_eigen(vectJoint.data(), vectJoint.size());
    state_representation::JointPositions actualJoinState = state_representation::JointPositions(robot_name,posJoint_eigen);  
    state_representation::Jacobian jacobianObject = model->compute_jacobian(actualJoinState); 
    MatrixXd jacobian = jacobianObject.data();

    return jacobian;
}

vector<double> RobotParameter::getIDynamics(vector<double> vectJoint, VectorXd speed_eigen){
    vector<double> speedJointNext;
    Matrix<double,6,1> twist = speed_eigen;

    Map<VectorXd> posJoint_eigen(vectJoint.data(), vectJoint.size());

    state_representation::JointPositions actualJoinState = state_representation::JointPositions(robot_name,posJoint_eigen);       
    state_representation::CartesianTwist nextPostwist = state_representation::CartesianTwist("nextPosTwist",twist);
    state_representation::JointVelocities nextJoinStateSpeed = model->inverse_velocity(nextPostwist,actualJoinState,paramsVel,tipJoint);
    VectorXd speedJointNext_eigen = nextJoinStateSpeed.data() ;
    for (int i = 0; i < speedJointNext_eigen.size(); ++i) {
        speedJointNext[i] = speedJointNext_eigen(i);
    }
    return speedJointNext;
}



//--------------------------------------IK-------------------------------------------------------------------------------

InverseKinematics::InverseKinematics() {  // Method/function defined inside the class
    baseLink   = "base_link";
    tipLink    = "wrist_3_link";
    URDF_param = "/robot_description";
    nJoint    = 6;
    vector<double> vector_0(nJoint, 0.0);
    posJointNext = vector_0;
    type = TRAC_IK::Distance;
    ikSolver= new TRAC_IK::TRAC_IK(baseLink, tipLink, URDF_param, timeoutInSecs, error, type);  
    
    valid = ikSolver->getKDLChain(chain);
    if (!valid) {
        ROS_ERROR("There was no valid KDL chain found");
    } 
}

pair<int, vector<double>> InverseKinematics::getIK(vector<double> actualJoint, vector<double> vectorQuatPos ) {  
    
    //Inverse kinematics trac-IK
    KDL::JntArray NextJointTask;
    KDL::JntArray actualJointTask; 

    VectorXd pos_joint_actual_eigen(nJoint);
    for(int i = 0 ;i<nJoint;++i){
        pos_joint_actual_eigen(i) =actualJoint[i];
    }
    actualJointTask.data = pos_joint_actual_eigen;

    KDL::Vector Vec(vectorQuatPos[4],vectorQuatPos[5],vectorQuatPos[6]);

    Quaterniond q(vectorQuatPos[3],vectorQuatPos[0],vectorQuatPos[1],vectorQuatPos[2]);
    q.normalize();
    KDL::Rotation Rot = KDL::Rotation::Quaternion(q.x(),q.y(),q.z(),q.w());
    KDL::Frame NextJointCartesian(Rot,Vec); 
    rc = ikSolver->CartToJnt(actualJointTask, NextJointCartesian, NextJointTask);
    if (rc< 0){
        ROS_INFO("no inverse kinematic found");    
    }

    VectorXd posJointNextEigen = NextJointTask.data;

    for(int i = 0 ;i<nJoint;++i){    
        posJointNext[i] =posJointNextEigen(i);
        }
    //msgP.data = posJointNext;
    
    //actualJointTask.data.clear();
    pair<int, vector<double>> myPair = make_pair(rc, posJointNext);

    return myPair;
    
} 

void InverseKinematics::updateIK(double err ,double timeoutInSecs ){
    ikSolver= new TRAC_IK::TRAC_IK(baseLink, tipLink, URDF_param, timeoutInSecs, err, type);  
}



void update_publisher_for_DS(RobotParameter &Rp,vector<double>  posJointActual,vector<double>  speedJointActual, ros::Publisher pos, ros::Publisher speed){

    vector<double> posCartActual = Rp.getFK(posJointActual);
    //publish state pos
    geometry_msgs::Pose poseActual;
    poseActual.orientation.x = posCartActual[0];
    poseActual.orientation.y = posCartActual[1];
    poseActual.orientation.z = posCartActual[2];
    poseActual.orientation.w = posCartActual[3];

    poseActual.position.x = posCartActual[4];
    poseActual.position.y = posCartActual[5];
    poseActual.position.z = posCartActual[6];


    pos.publish(poseActual);

    geometry_msgs::Twist twistActual =Rp.getTwist(posJointActual,speedJointActual);

    speed.publish(twistActual);
}




VectorXd speed_func(vector<double> Pos, vector<double> quat2,vector<double> speed){
    
     //orientation
    Vector4d q1,q2 ;
    q1 << Pos[0],Pos[1],Pos[2],Pos[3];
    q2 << quat2[0],quat2[1],quat2[2],quat2[3];

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
    
    vector<double> V = {Omega_out[0],Omega_out[1],Omega_out[2],speed[0],speed[1],speed[2]};

    double* pt = &V[0];
    VectorXd VOut = Map<VectorXd>(pt, 6);

    return VOut;
}

vector<double> Integral_func(vector<double> Pos_actual, VectorXd speed_actual, double dt, vector<double> quatDs)
{
    //Speed orientation integration
    
    Quaterniond q2(Pos_actual[0],Pos_actual[1],Pos_actual[2],Pos_actual[3]);
    //Vector3d euler = q.toRotationMatrix().eulerAngles(0, 1, 2);
    Vector3d Speedbis ={dt*speed_actual[0]/2,dt*speed_actual[1]/2,dt*speed_actual[2]/2};
    Quaterniond q1(Speedbis[0],Speedbis[1],Speedbis[2],1);

    Quaterniond resultQ;
    resultQ.setIdentity();

    resultQ.w() = q1.w() * q2.w() - q1.vec().dot(q2.vec());
    resultQ.vec() = q1.w() * q2.vec() + q2.w() * q1.vec() + q1.vec().cross(q2.vec());
    resultQ.normalize();
    resultQ.w() = q2.w() + resultQ.w();
    resultQ.vec() = q2.vec() + resultQ.vec();
    resultQ.normalize();

    //Speed position integration
    Vector3d speed_bis,past_bis,next_bis;
    speed_bis << speed_actual[3],speed_actual[4],speed_actual[5];
    past_bis << Pos_actual[4],Pos_actual[5],Pos_actual[6];
    next_bis =speed_bis*dt+ past_bis ;

    vector<double> pos_cart_Next;

    Quaterniond q(quatDs[3],quatDs[0],quatDs[1],quatDs[2]); //qw, qx, qy ,qz
    q.normalize();

    pos_cart_Next ={ q.x(),q.y(),q.z(),q.w(),next_bis[0],next_bis[1],next_bis[2]};

    return pos_cart_Next;
}

// Function that Calculate Root Mean Square
bool mseValue_cart(vector<double> v1, vector<double> v2,float tol)
{
    // tolerance of the errot between each point
    bool Reached = false;
    int crit =0;
    float err =0;
    int Len = v1.size();

    for (int i = 0; i < Len; i++) {
        err = err + (v1[i]-v2[i])*(v1[i]-v2[i]);
    }
    err = sqrt(err);
    if(err < tol){
        Reached =true;
    }

    return Reached;
}

