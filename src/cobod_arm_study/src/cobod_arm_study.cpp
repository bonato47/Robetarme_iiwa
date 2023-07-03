#include "pinocchio/fwd.hpp"
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Float64MultiArray.h"
#include "visualization_msgs/Marker.h"
#include <cmath>
#include <iostream>
#include <sstream>
#include <fstream>
#include <vector>
#include "std_srvs/Empty.h"
#include <trac_ik/trac_ik.hpp>
#include <eigen3/Eigen/Dense>
#include <filesystem>
#include <unistd.h>
#include "dynamical_systems/DynamicalSystemFactory.hpp"
#include "state_representation/space/cartesian/CartesianPose.hpp"
#include "state_representation/space/cartesian/CartesianTwist.hpp"
#include "state_representation/space/joint/JointPositions.hpp"
#include "state_representation/space/joint/JointVelocities.hpp"
#include "OsqpEigen/OsqpEigen.h" 
#include "robot_model/Model.hpp"

using namespace std;
using namespace Eigen;


//void CounterCallback(const sensor_msgs::JointState::ConstPtr);
bool mseValue(vector<double> , vector<double> , int);
vector<vector<double>> CSVtoVectorVectorDouble(string);
visualization_msgs::Marker printMarker(vector<double> quatPos, string base);

class inverseKin {       // The class
    public:             // Access specifier
        //iniailization Invers Kinematics
        string baseLink;
        string tipLink;
        string URDF_param="/robot_description";
        TRAC_IK::SolveType type =TRAC_IK::Speed;
        double error{};
        double timeoutInSecs{};
        int nJoint{};
        vector<double> posJointActual;
        VectorXd posJointActualEigen;
        vector<double> posJointNext;
        ros::V_string jointsName;
        bool init= false;
        TRAC_IK::TRAC_IK* ikSolver = nullptr;  


    void init_IK_iiwa() {  // Method/function defined inside the class
        this->baseLink  = "iiwa_link_0";
        this->tipLink   = "iiwa_link_ee";
        this->jointsName = {"iiwa_joint_1", "iiwa_joint_2", "iiwa_joint_3", "iiwa_joint_4", "iiwa_joint_5", "iiwa_joint_6", "iiwa_joint_7"};
        this->nJoint    = 7;
        this->init_general();
    }    
    void init_IK_cobod() {  // Method/function defined inside the class
        this->baseLink = "Link_1";
        this->tipLink = "Gripper_base";
        this->jointsName = {"Joint_1","Joint_2","Joint_3","Joint_5","Joint_6"};
        this->nJoint = 5;
        this->init_general();
    }
    void init_general(){

        vector<double> vector0(this->nJoint, 0.0);;
        this->posJointNext = vector0;
        this->posJointActual= vector0;
        this->ikSolver= new TRAC_IK::TRAC_IK(this->baseLink, this->tipLink, this->URDF_param, this->timeoutInSecs, this->error, this->type);  
        KDL::Chain chain;
        bool valid = ikSolver->getKDLChain(chain);
        if (!valid) {
        ROS_ERROR("There was no valid KDL chain found");
        } 
    }

    void getIK(vector<double> vectorQuatPos ) {  
        //Inverse kinematics trac-IK
        KDL::JntArray NextJointTask;
        KDL::JntArray actualJointTask; 
        Map<VectorXd> posJointNextEigen(&posJointNext[0], nJoint); 
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
     } 

    void CounterCallback(const sensor_msgs::JointState::ConstPtr msg)
    {
        posJointActual = msg->position;
        if(init == false){
            init = true;
        }    
        double* ptr = &posJointActual[0];
        posJointActualEigen = Map<VectorXd>(ptr, nJoint);
    }
};

int main(int argc, char **argv)
{
    sensor_msgs::JointState msgP;
    vector<vector<double>> traj_joint;

    //init ros
    ros::init(argc, argv, "Follow_traj");
    ros::NodeHandle Nh;

    //setup IK parameters
    inverseKin IK;
    string robot_name;
    Nh.getParam("/robot_name", robot_name);
    Nh.getParam("/errorIK",IK.error);
    Nh.getParam("/timeIK",IK.timeoutInSecs);

    if(robot_name == "iiwa7"){
        IK.init_IK_iiwa();
    } 
    else { 
        IK.init_IK_cobod();
    }
    robot_model::Model model(robot_name, "/home/ros/ros_ws/src/cobod_arm_study/urdf/iiwa7.urdf");

    //Initialisation of the Ros Node (Service, Subscrber and Publisher)
    ros::Subscriber sub = Nh.subscribe("joint_states", 1000, &inverseKin::CounterCallback, &IK);
    ros::Publisher pub = Nh.advertise<sensor_msgs::JointState>("joint_states", 1000);
    ros::Publisher visPub = Nh.advertise<visualization_msgs::Marker>("visualization_marker", 100 );
    //Frequency of the Ros loop

    ros::Rate loopRate(100);

    // Read trajectory from .csv 
    vector<vector<double>> traj_cart = CSVtoVectorVectorDouble("/home/ros/ros_ws/src/cobod_arm_study/src/Trajectory_Transform.csv");

    //waiting for the first joint position

     while(!IK.init){
        msgP.position = IK.posJointActual;//{0.0,0.0,0.0,0.0,0.0};
        msgP.velocity = {};
        msgP.effort   = {};     
        msgP.name = IK.jointsName;
        msgP.header.stamp=ros::Time::now();
        pub.publish(msgP);
        ros::spinOnce();
     } 
    
    ROS_INFO("Preparing trajectory...");

    std::ofstream myfile;
    myfile.open ("src/cobod_arm_study/src/trajectory_joints_cobod.csv");

    //Convert cartesian to joint space
    int size = int(traj_cart.size());
    for(int i = 0; i< size;i++)
    {
        //trac-ik inverse kinematic
        IK.getIK(traj_cart[i]);  

        traj_joint.push_back(IK.posJointNext);

        state_representation::Jacobian jac(robot_name, IK.nJoint, IK.tipLink,IK.baseLink);
        cout << jac << endl;
        std::stringstream ss;
        if(i > 1){
            for (auto it = IK.posJointNext.begin(); it != IK.posJointNext.end(); it++)    {
                if (it != IK.posJointNext.begin()) {
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

    myfile.close();
    //-------------------------------------------------------------------

    ROS_INFO("Trajectory well load, When you are ready press GO");
    string UserInput = "stop";
    while( UserInput != "GO"){
        cin >> UserInput;
    }
    //Next_pos = sensor_msgs::JointState
    //begin the Ros loop
    int Next  = 0;
    int count = 0 ;  

    while (ros::ok())
    {
        msgP.position = traj_joint[Next];
        msgP.velocity = {};
        msgP.effort   = {};     
        msgP.name = IK.jointsName;
        msgP.header.stamp=ros::Time::now();
        visualization_msgs::Marker markers = printMarker(traj_cart[Next], IK.baseLink);

        /* IK.posJointActual= traj_joint[Next];

        //ROS_INFO("%f", mseValue(pos,IK.posJointActual,n));
        if ((mseValue(IK.posJointActual,IK.posJointActual,IK.nJoint) && (Next < int(traj_cart.size()) )) || (count >50)){
            ++Next;
            if(Next == int(traj_cart.size())){
                ROS_INFO("Last point reached, stop ");
                return 0; 
            }
            //ROS_INFO("target reached, go next one ");
            count = 0;
        }   
        ++count;  */
       
        visPub.publish(markers);
        pub.publish(msgP);
        ros::spinOnce();
        loopRate.sleep();

        Next++;
        if(Next == int(traj_cart.size())){
            ROS_INFO("Last point reached, stop ");
            return 0;    }
    }
    return 0;
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
    }
    if(crit == 0){
        Reached =true;
    }

    return Reached;
}

vector<vector<double>> CSVtoVectorVectorDouble(string fname)
{
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


        vector<double> Line = {quat_x,quat_y,quat_z,quat_w,pos_x,pos_y,pos_z};
        Traj.push_back(Line);
    }

    return Traj;
}

visualization_msgs::Marker printMarker(vector<double> quatPos, string base){
    visualization_msgs::Marker marker;
    marker.header.frame_id = base;
    marker.header.stamp = ros::Time::now();
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    marker.ns= "ns";
    marker.id = 0;
    marker.pose.position.x =  quatPos[4];
    marker.pose.position.y =  quatPos[5];
    marker.pose.position.z =  quatPos[6];
    Eigen::Quaterniond quat = {quatPos[3],quatPos[0],quatPos[1],quatPos[2]};
     Eigen::Quaterniond Transf = {0,0.7,0,0.7};
    //Eigen::Quaterniond Transf = {0,0,0,1};

    Transf.normalize();

    quat =   quat * Transf;
    marker.pose.orientation.x =  quat.x();
    marker.pose.orientation.y =  quat.y();
    marker.pose.orientation.z =  quat.z();
    marker.pose.orientation.w =  quat.w();
    marker.scale.x = 0.3;
    marker.scale.y = 0.01;
    marker.scale.z = 0.01;
    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    ros::Duration one_seconds(1, 0);
    marker.lifetime = one_seconds; 
    return marker;
}
