#include "pinocchio/fwd.hpp"
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Float64MultiArray.h"
#include <cmath>
#include <iostream>
#include <sstream>
#include <fstream>
#include <vector>
#include "std_srvs/Empty.h"
#include <eigen3/Eigen/Dense>
#include <filesystem>
#include <unistd.h>
#include "trac_ik/trac_ik.hpp"
#include "Utils.h"
#include "dynamical_systems/DynamicalSystemFactory.hpp"
#include "state_representation/space/cartesian/CartesianPose.hpp"
#include "state_representation/space/cartesian/CartesianTwist.hpp"
#include "state_representation/space/joint/JointPositions.hpp"
#include "state_representation/space/joint/JointVelocities.hpp"
#include "OsqpEigen/OsqpEigen.h" 
#include "robot_model/Model.hpp"



using namespace Eigen;
using namespace std;

void CounterCallback(const sensor_msgs::JointState::ConstPtr);
bool mseValue(vector<double> , vector<double> , int);
vector<vector<double>> CSVtoVectorVectorDouble();
Eigen::Matrix< double,6,1> state_to_twist(Vector4d q1, Vector3d x01, Vector4d q2 ,Vector3d x02, double dt);

int n =7;
bool init= false;
vector<double> posJointActual(n);
vector<double> velJointActual(n);
vector<double> posJointNext(7);
vector<double> posDesJoint(n);
std_msgs::Float64MultiArray msgP;
vector<vector<double>> trajJoint;
double* ptr;
float pi =3.14;

int main(int argc, char **argv)
{
    double dt = 0;
    VectorXd posJointNext_eigen;
    VectorXd posJointActualEigen;

    //Initialisation of the Ros Node (Service, Subscrber and Publisher)
    ros::init(argc, argv, "Follow_traj_pinochio");
    ros::NodeHandle Nh;
    ros::Subscriber sub = Nh.subscribe("iiwa/joint_states", 1000, CounterCallback);
    ros::Publisher chatter_pub = Nh.advertise<std_msgs::Float64MultiArray>("iiwa/PositionController/command", 1000);
    //Frequency of the Ros loop
    ros::Rate loop_rate(200);

    // Read trajectory from .csv 
    vector<vector<double>> traj_cart = CSVtoVectorVectorDouble();

    //iniailization Invers Kinematics
    string robot_name = "iiwa7";

    robot_model::Model model(robot_name, "/home/ros/ros_ws/src/send_pos/urdf/iiwa7.urdf");

    ROS_INFO("Preparing trajectory...");

    std::ofstream myfile;
    myfile.open ("src/send_pos/src/trajectory_joints_Trajectory_Transform.csv");

    double timeout_in_secs=0.1;
    double error=1e-6; // a voir la taille 
    string IK = "init" ;
     //parameter for inverse velocities
    double alphaVel;
    double proportional_gain;
    double linear_velocity_limit; 
    double angular_velocity_limit=2;
     //parameter for inverse kinematics
    double damp;
    double alphaKin;
    double gamma;
    double margin;
    double tolerance;
    unsigned int maxNumberIteratons = 1000;

    while(IK == "init"){
        Nh.getParam("IK", IK);
        //parameter for inverse velocities
        Nh.getParam("/InversDynamics/alphaVel", alphaVel);
        Nh.getParam("/InversDynamics/proportional_gain", proportional_gain);
        Nh.getParam("/InversDynamics/linear_velocity_limit", linear_velocity_limit);
        Nh.getParam("/InversDynamics/angular_velocity_limit", angular_velocity_limit);
        cout << angular_velocity_limit << endl;

        //parameter for inverse kinematics
        Nh.getParam("/InversKinematics/damp", damp);
        Nh.getParam("/InversKinematics/alphaKin", alphaKin);
        Nh.getParam("/InversKinematics/gamma", gamma);
        Nh.getParam("/InversKinematics/margin", margin);
        Nh.getParam("/InversKinematics/tolerance", tolerance);
        ros::spinOnce();
    }

    std::chrono::nanoseconds t = 1000ns;
    struct robot_model::QPInverseVelocityParameters paramsVel = {alphaVel,proportional_gain,linear_velocity_limit,angular_velocity_limit,t};
    struct robot_model::InverseKinematicsParameters paramsKin = {damp,alphaKin,gamma,margin,tolerance,maxNumberIteratons };
    //waiting for the first joint position
    while(!init){
        ros::spinOnce();
        ptr = &posJointActual[0];
        posJointActualEigen = Map<VectorXd>(ptr, 7);
    }

    int size = int(traj_cart.size())-1;
    for(int i = 0; i< size;i++)
    {
        Vector3d p1, p2;
        Vector4d q1,q2;
        Quaterniond qa ;
        if(i == 0){
            ptr = &posJointActual[0];
            posJointActualEigen = Map<VectorXd>(ptr, 7); 

            if(IK != "kinematics"){
                state_representation::JointPositions actualJoinState =  state_representation::JointPositions(robot_name,posJointActualEigen);        
                state_representation::CartesianPose actualCartesianPose = model.forward_kinematics(actualJoinState,"iiwa_link_ee");
                Vector3d p1Prime = actualCartesianPose.get_position();
                Quaterniond q1Prime = actualCartesianPose.get_orientation();

                p1 = {p1Prime[0],p1Prime[1],p1Prime[2]};
                q1 = {q1Prime.x(),q1Prime.y(),q1Prime.z(),q1Prime.w()};
                p2 = {traj_cart[i][4],traj_cart[i][5],traj_cart[i][6]};            
                q2 = {traj_cart[i][0],traj_cart[i][1],traj_cart[i][2],traj_cart[i][3]};
                dt =  1;
            }
        }
        else{
            posJointActualEigen = posJointNext_eigen; 
            std::fill(posJointNext_eigen.begin(), posJointNext_eigen.end(), 0);
            p1 = {traj_cart[i][4],traj_cart[i][5],traj_cart[i][6]};
            p2 = {traj_cart[i+1][4],traj_cart[i+1][5],traj_cart[i+1][6]};
            q1 = {traj_cart[i][0],traj_cart[i][1],traj_cart[i][2],traj_cart[i][3]};
            q2 = {traj_cart[i+1][0],traj_cart[i+1][1],traj_cart[i+1][2],traj_cart[i+1][3]};
            //quaternion given by w,x,y,z
            qa = {traj_cart[i][3],traj_cart[i][0],traj_cart[i][1],traj_cart[i][2]};
            dt =  traj_cart[i+1][7];
        }
   
        if(IK == "kinematics"){
            //inverse kinematics--------------------------------------------
            state_representation::CartesianPose nextPosCart = state_representation::CartesianPose("nextPosCart",p1,qa);
            state_representation::JointPositions actualJoinState =  state_representation::JointPositions(robot_name,posJointActualEigen);        
            state_representation::JointPositions nextJoinStateKin = model.inverse_kinematics(nextPosCart,actualJoinState,paramsKin,"iiwa_link_ee");
            posJointNext_eigen = nextJoinStateKin.get_positions();
        }
        else{
            //inverse dynamics------------------------------------------------
            //convert pos and quat to twist
            Eigen::Matrix< double,6,1> twist = state_to_twist(q1, p1, q2, p2, dt);
            //cout << "posactual : " << p1 << endl;

            state_representation::JointPositions actualJoinState =  state_representation::JointPositions(robot_name,posJointActualEigen);        
            state_representation::CartesianTwist nextPostwist = state_representation::CartesianTwist("nextPostwist",twist);
            state_representation::JointVelocities nextJoinStateSpeed = model.inverse_velocity(nextPostwist,actualJoinState,paramsVel,"iiwa_link_ee");
            posJointNext_eigen = nextJoinStateSpeed.data()  * dt + posJointActualEigen;

/*              state_representation::JointPositions nextJoinState =  state_representation::JointPositions(robot_name,posJointNext_eigen);        
            state_representation::CartesianPose nextCartesianPose = model.forward_kinematics(nextJoinState,"iiwa_link_ee");
            Vector3d p1Prime = nextCartesianPose.get_position();
            Quaterniond q1Prime = nextCartesianPose.get_orientation();
            cout << "nextpos : " << p1Prime << endl;
            return 1;  */

        }
        
        //cout << "position from speed : " << posJointNext_eigen << endl;
       
        for(int i = 0 ;i<7;++i){
            posJointNext[i] =posJointNext_eigen(i);
        }
        trajJoint.push_back(posJointNext);
        std::stringstream ss;
        if(i > 15){
            for (auto it = posJointNext.begin(); it != posJointNext.end(); it++)    {
                if (it != posJointNext.begin()) {
                    ss << ",";
                }
                ss << *it;
            }
            myfile << ss.str();
            myfile <<"\n";
        }

        if(i == round(int(traj_cart.size())/2)){
            ROS_INFO("Half of the the trajectory load, please wait... ");
        }   
    }

    //-----------------------------------------
    myfile.close();

    ROS_INFO("Trajectory well load, When you are ready to go on the first position press start");
    string UserInput = "stop";
    while( UserInput != "start"){
        cin >> UserInput;
    }
    int Next  = 0;
    int count = 0 ;
    msgP.data = trajJoint[Next];
    posDesJoint= trajJoint[Next];
    chatter_pub.publish(msgP);

    while(Next == 0 ){
        if(mseValue(posJointActual,posDesJoint,n)|| (count >50)){
            ROS_INFO("first position reached, please Press GO when ready to shotcreet");
                while( UserInput != "GO"){
                    cin >> UserInput;
                }
            ++count;
            ++Next;
        }   
        ++count;

    } 
    count =1;
    //begin the Ros loop

    while (ros::ok())
    {
        msgP.data = trajJoint[Next];
        posDesJoint= trajJoint[Next];

        //ROS_INFO("%f", mseValue(pos,posDesJoint,n));
        if ((mseValue(posJointActual,posDesJoint,n) && (Next < size) ) || (count >50)){
            ++Next;
            if(Next == size){
                ROS_INFO("Last point reached, stop ");
                
                return 0; 
            }
            //ROS_INFO("target reached, go next one ");
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
    if(init == false){init = true;}
    posJointActual = msg->position;
    velJointActual = msg->velocity;
}

// Function that Calculate Root Mean Square
bool mseValue(vector<double> v1, vector<double> v2,int Num)
{
    // tolerance of the errot between each point
    float tol =0.05;
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


vector<vector<double>> CSVtoVectorVectorDouble()
{
    //string fname = "/home/ros/ros_ws/src/send_pos/src/trajectory_cart_short.csv";
    string fname = "/home/ros/ros_ws/src/send_pos/src/Trajectory_Transform.csv";

    vector<vector<double>> Traj;
    vector<vector<string>> content;
    vector<string> row;
    string line, word;

    fstream file (fname, ios::in);
    if(file.is_open())
    {
        while(getline(file, line))
        {
            row.clear();
            
            stringstream str(line);
            while(getline(str, word, ','))
                row.push_back(word);
            content.push_back(row);
        }
        ROS_INFO("file well readed");

    }
    else
        ROS_ERROR("Could not open the file\n");

    for(int i=0;i<int(content.size());i++) //
    {
        string::size_type sz;     // alias of size_t
        double quat_x = stod(content[i][0],&sz);
        double quat_y = stod(content[i][1],&sz);
        double quat_z = stod(content[i][2],&sz);
        double quat_w = stod(content[i][3],&sz);
        double pos_x = stod(content[i][4],&sz);
        double pos_y = stod(content[i][5],&sz);
        double pos_z = stod(content[i][6],&sz);
        double dt = stod(content[i][7],&sz);


        vector<double> Line = {quat_x,quat_y,quat_z,quat_w,pos_x,pos_y,pos_z,dt};
        Traj.push_back(Line);
    }

    return Traj;
}


 Matrix< double,6,1> state_to_twist(Vector4d q1, Vector3d x01, Vector4d q2 ,Vector3d x02, double dt)
{
    //position
    Vector3d lin_speed = (x02-x01)/dt;
    //orientation
    Vector4d dqd = Utils<double>::slerpQuaternion(q1, q2 ,dt);    
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
    

    Matrix< double,6,1> vOut;
    vOut << lin_speed[0],lin_speed[1],lin_speed[2],Omega_out[0],Omega_out[1],Omega_out[2];

    return vOut;
}


 