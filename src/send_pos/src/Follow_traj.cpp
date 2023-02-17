#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Float64MultiArray.h"
#include <cmath>
#include <iostream>
#include <sstream>
#include <fstream>
#include <vector>
#include "std_srvs/Empty.h"
#include <trac_ik/trac_ik.hpp>
#include <eigen3/Eigen/Dense>

using namespace std;

void CounterCallback(const sensor_msgs::JointState::ConstPtr);
bool mseValue(vector<double> , vector<double> , int);
vector<double> TakeLine(vector<vector<double>> , int );
vector<vector<double>> CSVtoVectorVectorDouble();

int n =7;
int Taille = 5000;
vector<double> eff(n);
vector<double> vel(n);
vector<double> pos_joint_actual(n);
float pi =3.14;
int main(int argc, char **argv)
{
    std_msgs::Float64MultiArray msgP;
    vector<double> pos_des_joint(n);
    vector<vector<double>> traj_joint;

    //Initialisation of the Ros Node (Service, Subscrber and Publisher)
    ros::init(argc, argv, "Follow_traj");
    ros::NodeHandle Nh;
    ros::Subscriber sub = Nh.subscribe("iiwa/joint_states", 1000, CounterCallback);
    ros::Publisher chatter_pub = Nh.advertise<std_msgs::Float64MultiArray>("iiwa/PositionController/command", 1000);
    //Frequency of the Ros loop
    ros::Rate loop_rate(200);

    // Read trajectory from .csv 
    vector<vector<double>> traj_cart = CSVtoVectorVectorDouble();

    //iniailization Invers Kinematics
    string base_link = "iiwa_link_0";
    string tip_link = "iiwa_link_ee";
    string URDF_param="/robot_description";
    double timeout_in_secs=0.05;
    double error=1e-3; // a voir la taille
    TRAC_IK::SolveType type=TRAC_IK::Distance;
    TRAC_IK::TRAC_IK ik_solver(base_link, tip_link, URDF_param, timeout_in_secs, error, type);  

    KDL::Chain chain;

    bool valid = ik_solver.getKDLChain(chain);
    if (!valid)
    {
        ROS_ERROR("There was no valid KDL chain found");
    }
    ROS_INFO("Preparing trajectory...");

    //Convert cartesian to joint space
    vector<double> pos_joint_next(7);
    double* ptr;
    for(int i = 0; i< int(traj_cart.size());i++)
    {
        KDL::JntArray Next_joint_task;
        KDL::JntArray actual_joint_task; 
        if(i == 0){
            ptr = &pos_joint_actual[0];
            Eigen::Map<Eigen::VectorXd> pos_joint_actual_eigen(ptr, 7); 
            actual_joint_task.data = pos_joint_actual_eigen; 
        }
        else {
            ptr = &pos_joint_next[0];
            Eigen::Map<Eigen::VectorXd> pos_joint_actual_eigen(ptr, 7); 
            actual_joint_task.data = pos_joint_actual_eigen; 
            std::fill(pos_joint_next.begin(), pos_joint_next.end(), 0);
        }
       
        KDL::Vector Vec(traj_cart[i][4],traj_cart[i][5],traj_cart[i][6]);
        KDL::Rotation Rot = KDL::Rotation::Quaternion(traj_cart[i][0],traj_cart[i][1],traj_cart[i][2],traj_cart[i][3]);
        //KDL::Rotation Rot = KDL::Rotation::Quaternion(1,1,1,1);
        KDL::Frame Next_joint_cartesian(Rot,Vec); 

        //ROS_INFO("%f" "%f" "%f" ,traj_cart[i][1],traj_cart[i][2],traj_cart[i][3]);

        Eigen::VectorXd pos_joint_next_eigen ;
        int rc = ik_solver.CartToJnt(actual_joint_task, Next_joint_cartesian, Next_joint_task);
        //ROS_INFO("%d",rc);
        pos_joint_next_eigen = Next_joint_task.data;
        for(int i = 0 ;i<7;++i){
            pos_joint_next[i] =pos_joint_next_eigen(i);
        }
        traj_joint.push_back(pos_joint_next);
        if(i == round(int(traj_cart.size())/2)){
            ROS_INFO("Half of the the trajectory load, please wait... ");
        }
    }
   
    //-----------------------------------------

    ROS_INFO("Trajectory well load, When you are ready press GO");
    string UserInput = "stop";
    while( UserInput != "GO"){
        cin >> UserInput;
    }

    //begin the Ros loop
    int Next  = 0;
    int count = 0 ;
    while (ros::ok())
    {
        msgP.data = traj_joint[Next];
        pos_des_joint= traj_joint[Next];

        //ROS_INFO("%f", mseValue(pos,pos_des_joint,n));
        if ((mseValue(pos_joint_actual,pos_des_joint,n) && (Next < int(traj_cart.size()) )) || (count >50)){
            ++Next;
            if(Next == int(traj_cart.size())){
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
    pos_joint_actual = msg->position;
    vel = msg->velocity;
    eff = msg->effort;
}

// Function that Calculate Root Mean Square
bool mseValue(vector<double> v1, vector<double> v2,int Num)
{
    // tolerance of the errot between each point
    float tol =0.1;
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

vector<double> TakeLine(vector<vector<double>> Mat, int numB )
{
    vector<double> vector(7);
    for (int i = 0; i < 7; i++) {
        vector[i] = Mat[numB][i];
    }

    return vector;
}

vector<vector<double>> CSVtoVectorVectorDouble()
{
    string fname = "/home/bonato/catkin_ws/src/send_pos/src/trajectory.csv";
    
    vector<vector<string>> content;
    vector<vector<double>> Traj;

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

    for(int i=0;i<Taille;i++) //int(content.size())
    {
        string::size_type sz;     // alias of size_t
        double pos_x = stod(content[i][0],&sz);
        double pos_y = stod(content[i][1],&sz);
        double pos_z = stod(content[i][2],&sz);
        double quad_x = stod(content[i][3],&sz);
        double quad_y = stod(content[i][4],&sz);
        double quad_z = stod(content[i][5],&sz);
        double quad_w = stod(content[i][6],&sz);

        vector<double> Line = {pos_x,pos_y,pos_z,quad_x,quad_y,quad_z,quad_w};//, Pos_y, Pos_z;
        Traj.push_back(Line);
    }
    return Traj;
}