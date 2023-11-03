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
#include <filesystem>
#include <unistd.h>

using namespace std;

void CounterCallback(const sensor_msgs::JointState::ConstPtr);
bool mseValue(vector<double> , vector<double> , int);
vector<double> TakeLine(vector<vector<double>> , int );
vector<vector<double>> CSVtoVectorVectorDouble(string fname);

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

    //Initialisation of the Ros Node (Service, Subscrber and Publisher)
    ros::init(argc, argv, "Follow_trajV2");
    ros::NodeHandle Nh;
    ros::Subscriber sub = Nh.subscribe("iiwa/joint_states", 1000, CounterCallback);
    ros::Publisher chatter_pub = Nh.advertise<std_msgs::Float64MultiArray>("iiwa/PositionController/command", 1000);
    //Frequency of the Ros loop
    ros::Rate loop_rate(200);

    // Read trajectory from .csv 
    string name_file;
    Nh.getParam("/Follow_trajV2/name", name_file);
    //${PWD}
        cout << name_file<< endl;
    name_file =  "/home/ros/ros_ws/src/send_pos/trajectories/" + name_file;
    cout << name_file<< endl;

    vector<vector<double>> traj_joint = CSVtoVectorVectorDouble(name_file);


    string UserInput = "stop";
    //begin the Ros loop
    int Next  = 0;
    int count = 0;
    while (ros::ok())
    {
        msgP.data = traj_joint[Next];
        pos_des_joint= traj_joint[Next];
   /*      if(pos_joint_actual[0] != 0 && count == -1){
            ROS_INFO("For first position press GO");
            while( UserInput != "GO"){
                cin >> UserInput;
            }
            UserInput = "stop";
            ++count;
        }else{

        }
        */
        if (count == 0 ){
            if (mseValue(pos_joint_actual,pos_des_joint,n) ){
                ROS_INFO("first position reached, please Press GO when ready to shotcreet");
                   while( UserInput != "GO"){
                        cin >> UserInput;
                   }
                ++count;
                ++Next;
            }   
        }
        if (count >= 1 ){
            if ((mseValue(pos_joint_actual,pos_des_joint,n) && (Next < int(traj_joint.size()) )) || (count >50)){
                ++Next;
                if(Next == int(traj_joint.size())){
                    ROS_INFO("Last point reached, stop ");
                    return 0; 
                }
                //ROS_INFO("target reached, go next one ");
                count = 0;
            }   
            ++count;
            chatter_pub.publish(msgP);
        }else{
            chatter_pub.publish(msgP);
        }

        ros::spinOnce();
        loop_rate.sleep();}
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
    float tol =0.2;
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
        double J1 = stod(content[i][0],&sz);
        double J2 = stod(content[i][1],&sz);
        double J3 = stod(content[i][2],&sz);
        double J4 = stod(content[i][3],&sz);
        double J5 = stod(content[i][4],&sz);
        double J6 = stod(content[i][5],&sz);
        double J7 = stod(content[i][6],&sz);

        vector<double> Line = {J1,J2,J3,J4,J5,J6,J7};//, Pos_y, Pos_z;
        Traj.push_back(Line);
    }

    return Traj;
}
