# ifndef SEND_POS_FUNC
# define SEND_POS_FUNC

#include "sensor_msgs/JointState.h"
#include <cmath>
#include <iostream>
#include <sstream>
#include <eigen3/Eigen/Dense>


using namespace std;
using namespace Eigen;

// Function that Calculate the speed with a DS
Vector3d speed_func(vector<double> Pos);

// Function that integrate the speed
Vector3d Integral_func(vector<double> Pos_actual, Vector3d speed_actual, double dt);

// Function that Calculate Root Mean Square
bool mseValue(vector<double> v1, vector<double> v2);

// Function that verify if goal is reached
bool Send_pos_func(vector<double> pos,vector<double> posDes);


#endif
