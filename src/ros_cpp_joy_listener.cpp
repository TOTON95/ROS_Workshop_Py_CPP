//ROS Headers
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <sensor_msgs/Joy.h>

//Header with generated message
#include <ros_workshop_py_cpp/Angles.h>

//Standard headers
#include <math.h>

#define PI 3.14159

//Joystick variables
bool btn_1;
double x_cmd;
double y_cmd;

//Custom message to hold angles
ros_workshop_py_cpp::Angles ang;

//Robot parameters
const double L1 = 82;
const double L2 = 62;
const double correction_S1 = 1;
const double correction_S2 = 1;
const double S1_correctionfactor = 8.3;
const double S2_correctionfactor = -20.2;
const double X_correctionfactor = 42;
const double Y_correctionfactor = -27;


//Function to acquire the data from a Joystick
void getJoy(const sensor_msgs::Joy::ConstPtr& button)
{
	btn_1 = button->buttons[0];
	x_cmd += button->axes[1];
	y_cmd += button->axes[0];
}

//Calculation of Inverse Kinematics 
void IK()
{
	//Mathematical operations
	double x = x_cmd + X_correctionfactor;
	double y = y_cmd + Y_correctionfactor;
	double c = sqrt(pow(x,2) + pow(y,2));
	double B = (acos((pow(L2,2) - pow(L1,2) - pow(c,2)) / (-2 * L1 * c))) * (180/PI);
	double C = (acos((pow(c,2) - pow(L2,2) - pow(L1,2)) / (-2 * L1 * L2))) * (180/PI);
	double theta = (asin(y/c)) * (180/PI);

	//Setting up the message
	ang.A = B + theta + S1_correctionfactor;
	ang.B = C + S2_correctionfactor;

	//Report to the current coordinates to USER
	ROS_INFO("Coordinates X: %lf Y: %lf \n",x, y); 	
}

int main(int argc, char** argv)
{
	//Init the node and give it a name
	ros::init(argc,argv,"drawing_arm");

	//Create a handle for node's operations
	ros::NodeHandle n;

	//Subscribers 
	ros::Subscriber joy_ctrl = n.subscribe("/joy",1000,getJoy); 

	//Publishers
	ros::Publisher rth_pub = n.advertise<std_msgs::Empty>("/rth",10);
	ros::Publisher ang_pub = n.advertise<ros_workshop_py_cpp::Angles>("/angles",10);

	//Message to return to home 
	std_msgs::Empty rth;

	//Set the rate to 100Hz
	ros::Rate r(100);

	while(n.ok())
	{

		//If btn_1 is pressed return to HOME
		if(btn_1)
		{
			//Send a message to the user
			ROS_INFO("Returning to HOME");

			//Send a message to RTH
			rth_pub.publish(rth);
		}

		else
		{
			//Calculate Inverse Kinectmatics (IK)
			IK();

			//Send the custom message with the angles
			ang_pub.publish(ang);
		}


		//Refresh the topics 
		ros::spinOnce();

		//Keep the desired rate	
		r.sleep();
	}
}
