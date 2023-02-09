#include "sensor_msgs/JointState.h"
#include <cmath>
#include <iostream>
#include <sstream>
#include <eigen3/Eigen/Dense>
#include "Send_pos_function.h"
#include <vector>

using namespace std;
using namespace Eigen;

Vector3d speed_func(vector<double> Pos)
{
    //int num_gridpoints = 30;
    Vector3d Position ;
    Position[0]= Pos[0];
    Position[1]= Pos[1];
    Position[2]= Pos[2];
    Matrix3d A;
    //Set a linear DS
    A << -1, 0, 0 ,0,-1,0,0,0,-1;
    Vector3d x01,b1,w; 
    // Set the attracotr
    x01 << 0.5,0.5,0.5;
    b1 =  -A*x01;
    w = A *Position +b1 ;
    //w = w.normalized()*0.1;
    return w;
}

Vector3d Integral_func(vector<double> Pos_actual, Vector3d speed_actual, double dt)
{
    Vector3d Position ;
    Position[0]= Pos_actual[0];
    Position[1]= Pos_actual[1];
    Position[2]= Pos_actual[2];
    Vector3d Pos_Next;
    Pos_Next = speed_actual *dt + Position;
    return Pos_Next;
}



// Function that Calculate Root Mean Square
bool mseValue(vector<double> v1, vector<double> v2)
{
    // tolerance of the errot between each point
    float tol =0.1;
    bool Reached = false;
    int crit =0;
    float err =0;

    for (int i = 0; i < 7; i++) {
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
bool Send_pos_func(vector<double> pos,vector<double> posDes)
{
    int count = 0 ;
    while (count < 50)
    {
        if (mseValue(pos,posDes) || (count >50)){
            return 0;
        }   
        ++count;
    }
    return 0;

}