#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"

#define RAD2DEG(x) ((x)*180./M_PI)															//Function to convert the radian reading into degrees.

ros::Publisher cmd_vel_pub_;																//Global variable to publish the movement variables.

void ScanAndMove(const sensor_msgs::LaserScan::ConstPtr& scan)                                                                                          //Creation of the function for the program
{
	geometry_msgs::Twist base_cmd;															//Object to the twist class
        base_cmd.linear.x=base_cmd.linear.y=base_cmd.angular.z=0;                                                                                       //Initialize all the movement variables																//Variable to store the last angle when encountered an obstacle
	float sum = 0;
	float mean = 0;
        bool status = 1;                                                                                                                                //Variable to change between linear and angular movements
        int available = 1;																//Variable to check if the program is available
        int count;																	//Count for the degrees
	while (available < 2)																//Check if the program is available
	{
                count = scan->scan_time / scan->time_increment;                                                                                         //Initialize the count variable
		for(int i = 0; i < count; i++)														//Start a "for-loop" to check all the data of the lidar
		{		
                        float degree = RAD2DEG(scan->angle_min + scan->angle_increment * i);                                                            //Convert the angle given by the lidar (radian) into degree
                        if ((degree > 122) and (degree < 238))                                                                                          //Check if the angle is in the range [149; 211] which corresponds to one need by the robot to pass between two obstacles at 50cm
			{
				if(scan->ranges[i] < 0.30000)												//Check if the distance detected is less than 30cm
				{
                                        status = 0;													//If it is the case, change the status variable and store a lastang value corresponding to the current angle
					sum = sum + (degree-180)*scan->ranges[i];
				}
			}
		}
		ROS_INFO ("Value : %f", sum);
                if (status == 1)															//Check the status of the robot (1=no obstacles, 0 = obstacles met)
		{
                        base_cmd.linear.x = 0.25;													//Set the movement variables for moving forward
			base_cmd.angular.z = 0;
                        ROS_INFO ("Move");														//Print  a message on the screen to have a visual information
			cmd_vel_pub_.publish(base_cmd);													//Publish the variables to start moving		
		}
		else												
		{
                        base_cmd.linear.x = 0;														//Stop the robot to moving forward
                        if(sum >= 0)															//Check if the last met angle is on the a left part or right part of the robot
			{
                                base_cmd.angular.z = 0.4;												//Set the variables for right rotation if lastang is on the left side
				ROS_INFO ("Left Rotation");												//Print  a message on the screen to have a visual information	
			}
			else
			{
                                base_cmd.angular.z = -0.4;												//Set the variables for left rotation if lastang is on the right side
                                ROS_INFO ("Right Rotation");												//Print  a message on the screen to have a visual information
			}
			cmd_vel_pub_.publish(base_cmd);													//Publish the twist commands
                        ros::Duration(0.2).sleep();													//Add a delay to the rotation to make it possible
		}
                base_cmd.angular.z = 0;															//Stop the robot rotation
		cmd_vel_pub_.publish(base_cmd);														//Publish the stop command
                available++;																//Set available to false to do the loop only once
	}
}


int main(int argc, char **argv)																//Declare the main function
{
    ros::init(argc, argv, "rplidar_node_client");                                                                                                       //Initialize the lidar node
        ros::NodeHandle nh_;																//Initialize the NodeHandle
        ros::Subscriber sub;																//Initialize the Subscriber
        cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi", 1);                                                               //Initialize the movement of the robot
        sub = nh_.subscribe<sensor_msgs::LaserScan>("/scan", 1000, ScanAndMove);                                                                        //Run the program
	ros::spin();

    	return 0;
}
