#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_srvs/Empty.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Point.h"
#include "iiwa_tools/GetIK.h"
#include "iiwa_tools/GetFK.h"
#include <cmath>
#include <iostream>
#include <sstream>
#include <Eigen/Dense>
#include <fstream>
#include <iterator>
#include <string>
#include <vector>
#include <algorithm>

using namespace std;
using namespace Eigen;

void CounterCallback(const sensor_msgs::JointState::ConstPtr);
int n =7;
vector<double> eff(n);
vector<double> vel(n);
vector<double> pos(n);
float pi =3.14;
vector<vector<double>> traj_joint;
vector<string> traj_joint_str;

struct CommaIterator
:
  public std::iterator<std::output_iterator_tag, void, void, void, void>
{
  std::ostream *os;
  std::string comma;
  bool first;

  CommaIterator(std::ostream& os, const std::string& comma)
  :
    os(&os), comma(comma), first(true)
  {
  }

  CommaIterator& operator++() { return *this; }
  CommaIterator& operator++(int) { return *this; }
  CommaIterator& operator*() { return *this; }
  template <class T>
  CommaIterator& operator=(const T& t) {
    if(first)
      first = false;
    else
      *os << comma;
    *os << t;
    return *this;
  }
};



int main(int argc, char **argv)
{
    iiwa_tools::GetIK  IK_state ;
    iiwa_tools::GetFK  FK_state ;

    vector<double> posDes(n);
    vector<double> V_temp(n);
    //Choose the position of the end effector {pos,quat}
    vector<vector<double>> traj_cart;

    //Initialisation of the Ros Node (Service, Subscrber and Publisher)
    ros::init(argc, argv, "Ds");
    ros::NodeHandle Nh;
    ros::ServiceClient client = Nh.serviceClient<iiwa_tools::GetIK>("iiwa/iiwa_ik_server");
    ros::ServiceClient client_FK = Nh.serviceClient<iiwa_tools::GetFK>("iiwa/iiwa_fk_server");
    ros::Subscriber sub = Nh.subscribe("iiwa/joint_states", 1000, CounterCallback);
    //ros::Publisher chatter_pub = Nh.advertise<std_msgs::Float64MultiArray>("iiwa/PositionController/command", 1000);
    //Frequency of the Ros loop
    ros::Rate loop_rate(10);
/*
    vector<geometry_msgs::Pose> h(Len_vec);
    for(int i = 0 ;i<Len_vec;++i){
        
        geometry_msgs::Point Pos_init ;
        Pos_init.x = traj_cart[i][0];
        Pos_init.y = traj_cart[i][1];
        Pos_init.z = traj_cart[i][2];
        geometry_msgs::Quaternion Quat_init ;
        Quat_init.x = traj_cart[i][3];
        Quat_init.y = traj_cart[i][4];
        Quat_init.z = traj_cart[i][5];
        Quat_init.w = traj_cart[i][6];

        geometry_msgs::Pose poses_init ;
        poses_init.position = Pos_init;
        poses_init.orientation = Quat_init;

        h[i] = (poses_init);
    }

    IK_state.request.poses = h;
    //Get IK to convert cartesian position to joint position

    client.call(IK_state);*/

    int Count = 0;
    //begin the Ros loop
    while (ros::ok())
    {
        //ROS_INFO("%f",traj_joint[Count][2]);
        ros::spinOnce();
        loop_rate.sleep();
        ++Count;
    }
    string filename = "./Test.txt";
    string path("/src/send_pos");
    std::vector<int> v(3,3);
    std::ostringstream oss;
    std::copy(v.begin(), v.end(), CommaIterator(oss, ","));
    std::string result = oss.str();
    const char *c_result = result.c_str();

    // Display the result;
    std::cout << c_result << "\n";
    return 0;
}

void CounterCallback(const sensor_msgs::JointState::ConstPtr msg)
{   
    pos = msg->position;
    vel = msg->velocity;
    eff = msg->effort;

    vector<double> V_temp(7);
    for(int j = 0 ;j<7;++j){
        V_temp.push_back(pos[j]);
    }
    traj_joint.push_back(V_temp);
}


