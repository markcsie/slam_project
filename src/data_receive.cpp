#include "ros/ros.h"
#include "../include/data_reader.h"
#include "slam_project/Robot_GroundTruth.h"
#include "slam_project/Robot_Odometry.h"
#include "slam_project/requestBarcode.h"

void publishMsg_callback3(const slam_project::Robot_GroundTruth& subMsg){
	cout<<"time: "<<subMsg.time<<endl;
}

void publishMsg_callback2(const slam_project::Robot_Odometry& subMsg){
//	cout<<"time: "<<subMsg.time<<" "<<subMsg.forward_velocity<<endl;
	int len = subMsg.num;
	if (len>1)
	    for (int i=0; i<len; i++)
	    	cout<<"subject: "<<i<<" range: "<<subMsg.range[i]<<endl;
}



int main(int argc, char **argv){
	//node name "subscriberMessage"
	ros::init(argc, argv, "subscriberMessage");
	ros::NodeHandle node;
	//1000 is the size of the message queue
	ros::Subscriber subscriber_color3 = node.subscribe("/publishMsg3", 1000, publishMsg_callback3); 
	ros::Subscriber subscriber_color2 = node.subscribe("/publishMsg2", 1000, publishMsg_callback2);

 	ros::ServiceClient client = node.serviceClient<slam_project::requestBarcode>("requestData");
	slam_project::requestBarcode srv;
	srv.request.a = 1;

	if (client.call(srv))
	{
	    int len = srv.response.barcode[0];
	    for (int i=1; i<=len; i++)
	    	ROS_INFO("Barcode: %d", srv.response.barcode[i]);
	}else{
	    ROS_ERROR("Failed to call service requestData");
	    return 1;
	}
	ros::spin();
	return 0;
}