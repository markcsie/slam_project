#include "ros/ros.h"
#include "../include/data_reader.h"
#include "slam_project/Robot_GroundTruth.h"
#include "slam_project/Robot_Odometry.h"

void publishMsg_callback3(const slam_project::Robot_GroundTruth& subMsg){
	cout<<"time: "<<subMsg.time<<endl;
}

void publishMsg_callback2(const slam_project::Robot_Odometry& subMsg){
	cout<<"time: "<<subMsg.time<<" "<<subMsg.forward_velocity<<endl;
}



int main(int argc, char **argv){
	ros::init(argc, argv, "subscriberMessage");
	ros::NodeHandle node;
	ros::Subscriber subscriber_color3 = node.subscribe("/publishMsg3", 1000, publishMsg_callback3);
	ros::Subscriber subscriber_color2 = node.subscribe("/publishMsg2", 1000, publishMsg_callback2);
	ros::spin();
	return 0;
}