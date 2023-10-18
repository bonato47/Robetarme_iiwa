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
        double errorMax{};
        double timeoutInSecs{};
        int nJoint{};
        vector<double> posJointActual;
        vector<double> velJointActual;
        vector<double> posJointNext;
        VectorXd velJointActualEigen;
        VectorXd posJointActualEigen;
        VectorXd posJointNextEigen;
        ros::V_string jointsName;
        bool init= false;
        TRAC_IK::TRAC_IK* ikSolver = nullptr;  


    void init_IK_iiwa() {  // Method/function defined inside the class
        baseLink  = "iiwa_link_0";
        tipLink   = "iiwa_link_ee";
        jointsName = {"iiwa_joint_1", "iiwa_joint_2", "iiwa_joint_3", "iiwa_joint_4", "iiwa_joint_5", "iiwa_joint_6", "iiwa_joint_7"};
        nJoint    = 7;
        init_general();
    }    
    void init_IK_cobod() {  // Method/function defined inside the class
        baseLink = "Link_1";
        tipLink = "Gripper_base";
        jointsName = {"Joint_1","Joint_2","Joint_3","Joint_5","Joint_6"};
        nJoint = 5;
        init_general();
    }
     
    void init_IK_cobod_6DOF() {  // Method/function defined inside the class
        baseLink = "Link_1";
        tipLink = "Gripper_base";
        jointsName = {"Joint_1","Joint_2","Joint_3","Joint_4","Joint_5","Joint_6"};
        nJoint = 6;
        init_general();
    }
    void init_IK_ur5() {  // Method/function defined inside the class
        baseLink = "base_link";
        tipLink = "ee_link";
        jointsName = {"shoulder_pan_joint","shoulder_lift_joint","elbow_joint","wrist_1_joint","wrist_2_joint","wrist_3_joint"};
        nJoint = 6;
        init_general();
    }
    void init_IK_ur10() {  // Method/function defined inside the class
        baseLink = "base_link";
        tipLink = "tool0";
        jointsName = {"shoulder_pan_joint","shoulder_lift_joint","elbow_joint","wrist_1_joint","wrist_2_joint","wrist_3_joint"};
        nJoint = 6;
        init_general();
    }
    void init_general(){
        vector<double> vector0(nJoint, 0.0);;
        posJointNext = vector0;
        posJointActual= vector0;
        velJointActual= vector0;
        posJointNextEigen = Map<VectorXd>(&posJointNext[0], nJoint);
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

    void CounterCallback(const sensor_msgs::JointState::ConstPtr msg)
    {
        posJointActual = msg->position;
        velJointActual = msg->velocity;

        if(init == false){
            init = true;
        }    
        posJointActualEigen = Map<VectorXd>(&posJointActual[0], nJoint);
        velJointActualEigen = Map<VectorXd>(&velJointActual[0], nJoint); 
    }
};

int main(int argc, char **argv)
{
    string name_path = "";
    sensor_msgs::JointState msgP;
    vector<vector<double>> traj_joint;

    //init ros
    ros::init(argc, argv, "Follow_traj");
    ros::NodeHandle Nh;

    //setup IK parameters
    inverseKin IK;
    string robot_name;
    Nh.getParam("/robot_name", robot_name);
    Nh.getParam("/errorIK_min",IK.error);
    Nh.getParam("/errorIK_max",IK.errorMax);

    Nh.getParam("/timeIK",IK.timeoutInSecs);

    if(robot_name == "iiwa7"){
        IK.init_IK_iiwa();
    } 
    else if(robot_name == "cobod_arm") { 
        IK.init_IK_cobod();
    }
    else if(robot_name == "ur5") { 
        IK.init_IK_ur5();
    }
    else if(robot_name == "ur10") { 
        IK.init_IK_ur10();
    }
    else if(robot_name == "cobod_arm_6DOF") { 
        IK.init_IK_cobod_6DOF();
    }

    string path_urdf = "/home/ros/ros_ws/src/cobod_arm_study/urdf/" + robot_name + ".urdf";
    robot_model::Model model(robot_name,path_urdf);

    //Initialisation of the Ros Node (Service, Subscrber and Publisher)
    ros::Subscriber sub = Nh.subscribe("joint_states", 1000, &inverseKin::CounterCallback, &IK);
    ros::Publisher pub = Nh.advertise<sensor_msgs::JointState>("joint_states", 1000);
    ros::Publisher visPub = Nh.advertise<visualization_msgs::Marker>("visualization_marker", 100 );
    //Frequency of the Ros loop

    ros::Rate loopRate(100);

    // Read trajectory from .csv 
    vector<vector<double>> traj_cart = CSVtoVectorVectorDouble("/home/ros/ros_ws/src/cobod_arm_study/src/csv_cobod/s_shape_trajectory_ur10.csv");

    //waiting for the first joint position
     while(!IK.init){
        msgP.position = IK.posJointActual;
        msgP.velocity = {};
        msgP.effort   = {};     
        msgP.name = IK.jointsName;
        msgP.header.stamp=ros::Time::now();
        pub.publish(msgP);
        ros::spinOnce();
     } 
    
    ROS_INFO("Preparing trajectory...");

    std::ofstream myfile;
    string nameJoint = "/home/ros/ros_ws/src/cobod_arm_study/src/trajectories/trajectory_joints_" + robot_name + name_path +"error"+ to_string(IK.error)+"time"+ to_string(IK.timeoutInSecs) +".csv";

    myfile.open(nameJoint);

    //Convert cartesian to joint space
    int size = int(traj_cart.size());
    for(int i = 0; i< size;i++)
    {
        //trac-ik inverse kinematic
        double err = IK.error;
        IK.updateIK(IK.error);
        int rc = IK.getIK(traj_cart[i]);
        while(rc  < 0){
            err= err*5;
            IK.updateIK(err);
            rc = IK.getIK(traj_cart[i]);
        }
 
        traj_joint.push_back(IK.posJointNext);

        vector<double> output_csv;
        //compute jaocbian to have the manipulability matrix
        state_representation::JointPositions  actualJointPos =  state_representation::JointPositions(robot_name,IK.posJointNextEigen);        
        state_representation::Jacobian jacobian = model.compute_jacobian(actualJointPos);

        Eigen::MatrixXd manipulabilityEigen =jacobian.data()*jacobian.transpose().data();
        Eigen::EigenSolver<Eigen::MatrixXd> eigensolver;

        //manipulability
        eigensolver.compute(manipulabilityEigen);
        Eigen::VectorXd eigen_values = eigensolver.eigenvalues().real();  
        
        output_csv.push_back(eigen_values(0));
        output_csv.push_back(eigen_values(1));
        output_csv.push_back(eigen_values(2));

      
        //force
        eigensolver.compute(manipulabilityEigen.inverse());
        Eigen::VectorXd eigen_values_force = eigensolver.eigenvalues().real();  
        output_csv.push_back(eigen_values_force(0));
        output_csv.push_back(eigen_values_force(1));
        output_csv.push_back(eigen_values_force(2));          
 
        //error
        state_representation::CartesianPose nextCartesianPose = model.forward_kinematics(actualJointPos,IK.tipLink);
        Vector3d p1Prime = nextCartesianPose.get_position();

        Vector3d p2Prime = {traj_cart[i][4],traj_cart[i][5],traj_cart[i][6]} ;
        Vector3d errorEigen =  p1Prime-p2Prime;

        output_csv.push_back(errorEigen.norm());

        Quaterniond q1Prime = nextCartesianPose.get_orientation();
        Quaterniond q2Prime = {traj_cart[i][3],traj_cart[i][0],traj_cart[i][1],traj_cart[i][2]};
        Quaterniond errorQuatEigen =  q1Prime.inverse()*q2Prime;
        errorQuatEigen.normalize();

        // Prendre en compte le modulo 2π
        double angle = 2.0 * std::atan2(errorQuatEigen.vec().norm(), std::abs(errorQuatEigen.w()));
        // Ramener l'angle dans l'intervalle [0, 2π]
        if (angle > M_PI) {
            angle = 2.0 * M_PI - angle;
        }

        output_csv.push_back(angle); 
        output_csv.push_back(p1Prime(0));
        output_csv.push_back(p1Prime(1));
        output_csv.push_back(p1Prime(2));  
 
        std::stringstream ss;
        if(i > 1){
            for (auto it = output_csv.begin(); it != output_csv.end(); it++)    {
                if (it != output_csv.begin()) {
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
        Quaterniond q(quat_w,quat_x,quat_y,quat_z);
        q.normalize();


        vector<double> Line = {q.x(),q.y(),q.z(),q.w(),pos_x,pos_y,pos_z};
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
