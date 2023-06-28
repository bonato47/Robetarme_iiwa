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

using namespace std;
using namespace Eigen;


//void CounterCallback(const sensor_msgs::JointState::ConstPtr);
bool mseValue(vector<double> , vector<double> , int);
vector<vector<double>> CSVtoVectorVectorDouble(string);
visualization_msgs::Marker printMarker(vector<double> quatPos, string base);

class inverseKin {       // The class
    public:             // Access specifier
        //iniailization Invers Kinematics
        string base_link;
        string tip_link ;
        string URDF_param="/robot_description";
        TRAC_IK::SolveType type =TRAC_IK::Speed;
        double error;
        double timeout_in_secs;
        int n_joint;
        vector<double> posJointActual;
        VectorXd posJointActualEigen;
        vector<double> pos_joint_next;
        vector<double> pos_des_joint;
        ros::V_string jointsName;
        bool init= false;

    void init_IK_iiwa() {  // Method/function defined inside the class
        base_link = "iiwa_link_0";
        tip_link = "iiwa_link_7";
        n_joint =7;
        posJointActual.reserve(n_joint);
        pos_des_joint.reserve(n_joint);
        pos_joint_next.reserve(n_joint);
        jointsName ={"iiwa_joint_1", "iiwa_joint_2", "iiwa_joint_3", "iiwa_joint_4", "iiwa_joint_5", "iiwa_joint_6", "iiwa_joint_7"};
    }    
    void init_IK_cobod() {  // Method/function defined inside the class
        base_link = "Link_1";
        tip_link = "Gripper_base";
        n_joint = 5;
        posJointActual.reserve(n_joint);
        pos_des_joint.reserve(n_joint);
        pos_joint_next.reserve(n_joint);
        jointsName = {"Joint_1","Joint_2","Joint_3","Joint_5","Joint_6"};
    }

    void updateParamIK( double errorUpdate,double timeUpdate) {  // Method/function defined inside the class
        error = errorUpdate;
        timeout_in_secs = timeUpdate;
        TRAC_IK::TRAC_IK ik_solver(base_link, tip_link, URDF_param, timeout_in_secs, error, type);  
    }

    void getIK( ) {  // Method/function defined inside the class
        TRAC_IK::TRAC_IK ik_solver(base_link, tip_link, URDF_param, timeout_in_secs, error, type);  
         KDL::Chain chain;
        bool valid = ik_solver.getKDLChain(chain);
        if (!valid)
        {
            ROS_ERROR("There was no valid KDL chain found");
        }
    }

    void CounterCallback(const sensor_msgs::JointState::ConstPtr msg)
    {
        posJointActual = msg->position;
        if(init == false){
            init = true;
        }    
        double* ptr = &posJointActual[0];
        posJointActualEigen = Map<VectorXd>(ptr, n_joint);
    }
};

int main(int argc, char **argv)
{
    sensor_msgs::JointState msgP;
    vector<vector<double>> traj_joint;

    //Initialisation of the Ros Node (Service, Subscrber and Publisher)
    ros::init(argc, argv, "Follow_traj");
    ros::NodeHandle Nh;
    inverseKin IK;
    ros::Subscriber sub = Nh.subscribe("joint_states", 1000, &inverseKin::CounterCallback, &IK);
    ros::Publisher pub = Nh.advertise<sensor_msgs::JointState>("joint_states", 1000);
    ros::Publisher vis_pub = Nh.advertise<visualization_msgs::Marker>("visualization_marker", 100 );
    //Frequency of the Ros loop
    ros::Rate loop_rate(100);


    //setup IK parameters
    string robot_name;
    Nh.getParam("/robot_name", robot_name);
    if(robot_name == "iiwa"){
        IK.init_IK_iiwa();
    } 
    else { 
        IK.init_IK_cobod();
    }
    Nh.getParam("/errorIK",IK.error);
    Nh.getParam("/timeIK",IK.timeout_in_secs);


    // Read trajectory from .csv 
    vector<vector<double>> traj_cart = CSVtoVectorVectorDouble("/home/ros/ros_ws/src/send_pos/src/Trajectory_Transform.csv");
    vector<vector<double>> traj_marker = CSVtoVectorVectorDouble("/home/ros/ros_ws/src/send_pos/src/Trajectory_Transform_noend.csv");

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

    TRAC_IK::TRAC_IK ik_solver(IK.base_link, IK.tip_link, IK.URDF_param, IK.timeout_in_secs, IK.error, IK.type);  
    KDL::Chain chain;
    bool valid = ik_solver.getKDLChain(chain);
    if (!valid)
    {
        ROS_ERROR("There was no valid KDL chain found");
    } 
    ROS_INFO("Preparing trajectory...");

    std::ofstream myfile;
    myfile.open ("src/send_pos/src/trajectory_joints_cobod.csv");

    //Convert cartesian to joint space
    int size = int(traj_cart.size());
    vector<double> pos_joint_next(IK.n_joint);
    for(int i = 0; i< size;i++)
    {
        KDL::JntArray Next_joint_task;
        KDL::JntArray actual_joint_task; 
        VectorXd pos_joint_next_eigen ;

        if(i == 0){
            actual_joint_task.data = IK.posJointActualEigen; 
        }
        else {
            Map<VectorXd> pos_joint_next_eigen(&pos_joint_next[0], IK.n_joint); 
            actual_joint_task.data = pos_joint_next_eigen; 
            std::fill(pos_joint_next.begin(), pos_joint_next.end(), 0);
        }  

        KDL::Vector Vec(traj_cart[i][4],traj_cart[i][5],traj_cart[i][6]);
        KDL::Rotation Rot = KDL::Rotation::Quaternion(traj_cart[i][0],traj_cart[i][1],traj_cart[i][2],traj_cart[i][3]);
        KDL::Frame Next_joint_cartesian(Rot,Vec); 
        int rc = ik_solver.CartToJnt(actual_joint_task, Next_joint_cartesian, Next_joint_task);
        pos_joint_next_eigen = Next_joint_task.data;
        for(int i = 0 ;i<IK.n_joint;++i){
            pos_joint_next[i] =pos_joint_next_eigen(i);

        }
        traj_joint.push_back(pos_joint_next);


        std::stringstream ss;
        if(i > 1){
            for (auto it = pos_joint_next.begin(); it != pos_joint_next.end(); it++)    {
                if (it != pos_joint_next.begin()) {
                    ss << ",";
                }
                ss << *it;
            }
            myfile << ss.str();
            myfile <<"\n";
        }

 /*     if(rc < 0){
            ROS_INFO("no solution found");
            myfile.close();
            break;
        }

  */

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
        visualization_msgs::Marker markers = printMarker(traj_cart[Next], IK.base_link);

        /* IK.posJointActual= traj_joint[Next];

        //ROS_INFO("%f", mseValue(pos,IK.posJointActual,n));
        if ((mseValue(IK.posJointActual,IK.posJointActual,IK.n_joint) && (Next < int(traj_cart.size()) )) || (count >50)){
            ++Next;
            if(Next == int(traj_cart.size())){
                ROS_INFO("Last point reached, stop ");
                return 0; 
            }
            //ROS_INFO("target reached, go next one ");
            count = 0;
        }   
        ++count;  */
       
        vis_pub.publish(markers);
        pub.publish(msgP);
        ros::spinOnce();
        loop_rate.sleep();

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
       // ROS_INFO("%f",err);

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
