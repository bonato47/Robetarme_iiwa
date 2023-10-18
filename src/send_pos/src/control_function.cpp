#include <cmath>
#include <iostream>
#include <sstream>
#include <eigen3/Eigen/Dense>
#include "control_function.h"
#include <vector>

#include "Utils.h"


using namespace std;
using namespace Eigen;



ActualState::ActualState(ros::ServiceClient FK){
    client_FK = FK;
    vector<double> vector0(nJoint, 0.0);
    posJointActual   = vector0;
    posCartActual    = vector0;
    speedJointActual = vector0;
    joint_std64.data = {posJointActual[0],posJointActual[1],posJointActual[2],posJointActual[3],posJointActual[4],posJointActual[5],posJointActual[6]};
    initFK();
    }
void ActualState::initFK(){
        //iniailization Forward Kinematics
    FK_state.request.joints.layout.dim.push_back(std_msgs::MultiArrayDimension());
    FK_state.request.joints.layout.dim.push_back(std_msgs::MultiArrayDimension());
    FK_state.request.joints.layout.dim[0].size = 1;
    FK_state.request.joints.layout.dim[1].size = nJoint;
}
vector<double> ActualState::getFK(vector<double> vectJoint){
    // Take joints state actual and convert to cartesian state with the help of th FK service
    std_msgs::Float64MultiArray joint;
    geometry_msgs::Quaternion actualCartQuat;
    geometry_msgs::Point actualCartPos;
    joint.data = {vectJoint[0],vectJoint[1],vectJoint[2],vectJoint[3],vectJoint[4],vectJoint[5],vectJoint[6]};
    FK_state.request.joints.data =  joint.data;
    client_FK.call(FK_state);
    actualCart= FK_state.response.poses[0], FK_state.response.poses[1];
    actualCartPos = actualCart.position;
    actualCartQuat = actualCart.orientation;
    vector<double> posCartActualTemp = {actualCartQuat.x,actualCartQuat.y,actualCartQuat.z,actualCartQuat.w,actualCartPos.x,actualCartPos.y,actualCartPos.z};
    return posCartActualTemp;
}

void ActualState::CounterCallback(const sensor_msgs::JointState::ConstPtr msg)
{
    posJointActual = msg->position;
    speedJointActual = msg->velocity;
    joint_std64.data = {posJointActual[0],posJointActual[1],posJointActual[2],posJointActual[3],posJointActual[4],posJointActual[5],posJointActual[6]};

    if(initCheck == false){
        initCheck = true;
    }    
}



NextState::NextState(double err,double t) {  // Method/function defined inside the class
    baseLink  = "iiwa_link_0";
    tipLink   = "iiwa_link_ee";
    nJoint    = 7;
    error=err;
    timeoutInSecs = t;
    init_general();
}    

void NextState::init_general(){
    vector<double> vector0_4(4, 0.0);
    quatFromDS = vector0_4;
    vector<double> vector0_3(3, 0.0);
    speedFromDS = vector0_3;
    vector<double> vector0_7(7, 0.0);
    posJointNext = vector0_7;

    ikSolver= new TRAC_IK::TRAC_IK(baseLink, tipLink, URDF_param, timeoutInSecs, error, type);  
    KDL::Chain chain;
    bool valid = ikSolver->getKDLChain(chain);
    if (!valid) {
        ROS_ERROR("There was no valid KDL chain found");
    } 
}

int NextState::getIK(vector<double> actualJoint, vector<double> vectorQuatPos ) {  
    //convert to eigen
    int L = actualJoint.size();
    double* pt = &actualJoint[0];
    //Inverse kinematics trac-IK
    KDL::JntArray NextJointTask;
    KDL::JntArray actualJointTask; 
    actualJointTask.data = Map<VectorXd>(pt, L);
    KDL::Vector Vec(vectorQuatPos[4],vectorQuatPos[5],vectorQuatPos[6]);

    Quaterniond q(vectorQuatPos[3],vectorQuatPos[0],vectorQuatPos[1],vectorQuatPos[2]);
    q.normalize();
    KDL::Rotation Rot = KDL::Rotation::Quaternion(q.x(),q.y(),q.z(),q.w());
    KDL::Frame NextJointCartesian(Rot,Vec); 
    int rc = ikSolver->CartToJnt(actualJointTask, NextJointCartesian, NextJointTask);
    if (rc< 0){
        ROS_INFO("no inverse kinematic found");    
    }

    VectorXd posJointNextEigen = NextJointTask.data;

    for(int i = 0 ;i<nJoint;++i){    
        posJointNext[i] =posJointNextEigen(i);
        }
    msgP.data = posJointNext;
    
    return rc;
    
} 

void NextState::updateIK(double err){
        ikSolver= new TRAC_IK::TRAC_IK(baseLink, tipLink, URDF_param, timeoutInSecs, err, type);  
}

void NextState::poseCallback(const geometry_msgs::Pose::ConstPtr& msg) {
    // Process the received Pose message here
    // For example, you can access the position and orientation as follows:
    double x = msg->position.x;
    double y = msg->position.y;
    double z = msg->position.z;

    double qx = msg->orientation.x;
    double qy = msg->orientation.y;
    double qz = msg->orientation.z;
    double qw = msg->orientation.w;


    speedFromDS ={x,y,z};
    quatFromDS = {qx,qy,qz,qw};

    if(initCheck == false && qx != 0 && qy != 0 && qz != 0 && qw != 0){
        initCheck = true;
    }
}


void Fk_and_pub(ActualState& actu, ros::ServiceClient CL, ros::Publisher pos, ros::Publisher speed){

    actu.posCartActual = actu.getFK(actu.posJointActual);
    //publish state pos
    pos.publish(actu.actualCart);
    geometry_msgs::Twist twistActual = get_twist_fromService(actu.posJointActual,actu.speedJointActual,CL);

    speed.publish(twistActual);
}


void update_publisher_for_DS(ActualState& actu,vector<double> vectorJoint, ros::Publisher pos, ros::Publisher speed,ros::ServiceClient CL,ros::Rate loop_rate){
    //FK
    actu.posCartActual = actu.getFK(vectorJoint);
    //publish state pos    ros::spinOnce();        
    loop_rate.sleep();
    pos.publish(actu.actualCart);

    geometry_msgs::Twist twistActual = get_twist_fromService(vectorJoint,actu.speedJointActual,CL);

    speed.publish(twistActual);

    return;
}



geometry_msgs::Twist get_twist_fromService(vector<double> posJoint, vector<double> speedJoint,ros::ServiceClient client){

    // Create a Twist message
    geometry_msgs::Twist twist;

    iiwa_tools::GetJacobian srv;
    srv.request.joint_angles = posJoint;
    srv.request.joint_velocities = speedJoint;

    client.call(srv);
    vector<double> jac = srv.response.jacobian.data;

    // Reshape the multi_array into an Eigen matrix
    int rows = 6; // Specify the desired number of rows
    int cols = 7; // Specify the desired number of columns
    double* pt = &jac[0];
    if (pt == nullptr){
        return twist;   
    }
    MatrixXd jacMatrix = Map<MatrixXd>(pt, rows, cols);
    
    double* pt2 = &speedJoint[0]; 
    VectorXd speedJointEigen = Map<VectorXd>(pt2, 7);


    
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

vector<double> Integral_func(vector<double> Pos_actual, VectorXd speed_actual, double dt, NextState& next)
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

    Quaterniond q(next.quatFromDS[3],next.quatFromDS[0],next.quatFromDS[1],next.quatFromDS[2]);
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

