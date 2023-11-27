# ifndef CONTROL_FUNC
# define CONTROL_FUNC

#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose.h"
#include "iiwa_tools/GetFK.h"
#include "iiwa_tools/GetJacobian.h" 
#include "control_msgs/JointTrajectoryControllerState.h"
#include "control_msgs/FollowJointTrajectoryActionGoal.h"
#include <trac_ik/trac_ik.hpp>
#include <eigen3/Eigen/Dense>



using namespace std;
using namespace Eigen;


class ActualState {       // The class
  public:             // Access specifier
    int nJoint =7;
    bool initCheck= false;
    std_msgs::Float64MultiArray joint_std64;

    geometry_msgs::Pose actualCart;
    vector<double> posJointActual;
    vector<double> posCartActual;
    vector<double> speedJointActual;
    vector<double> speedCartActual;

    ros::ServiceClient client_FK;
    iiwa_tools::GetFK  FK_state ;

    ActualState(ros::ServiceClient FK);
    void initFK();
    vector<double> getFK(vector<double> vectJoint);
    void CounterCallback(const sensor_msgs::JointState::ConstPtr msg);
};

class NextState {       // The class
    public:             // Access specifier
        //iniailization Invers Kinematics
        string baseLink;
        string tipLink;
        string URDF_param="/robot_description";
        TRAC_IK::SolveType type =TRAC_IK::Distance;
        double error; 
        double timeoutInSecs;
        int nJoint{};

        std_msgs::Float64MultiArray msgP;
        vector<double> posJointNext;
        vector<double> speedFromDS;
        vector<double> quatFromDS;

        bool initCheck= false;
        TRAC_IK::TRAC_IK* ikSolver = nullptr;  

    NextState(double err = 0.001,double t = 0.05);
    
    void init_general();
    int getIK(vector<double> actualJoint, vector<double> vectorQuatPos ) ;

     void updateIK(double err);

    void poseCallback(const geometry_msgs::Pose::ConstPtr& msg) ;

};

//do fk and publish
void Fk_and_pub(ActualState& actu, ros::ServiceClient CL, ros::Publisher pos, ros::Publisher speed);

//return the twist speed
geometry_msgs::Twist get_twist_fromService(vector<double> posJoint, vector<double> speedJoint,ros::ServiceClient client);

//Function that update the publisher for the DS and publish
void update_publisher_for_DS(ActualState& actu,vector<double> vectorJoint, ros::Publisher pos, ros::Publisher speed,ros::ServiceClient client,ros::Rate loop_rate);

// Function that Calculate the speed with a DS
VectorXd speed_func(vector<double> Pos, vector<double> quat2,vector<double> speed);

// Function that integrate the speed
vector<double> Integral_func(vector<double> Pos_actual, VectorXd speed_actual, double dt,NextState& next);

// Function that Calculate Root Mean Square
bool mseValue_cart(vector<double> v1, vector<double> v2, float tol);


#endif
