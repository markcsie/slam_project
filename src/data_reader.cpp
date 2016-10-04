#include <ros/ros.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "data_reader");
  ros::NodeHandle node;
       
  ros::Rate rate(1.0);
  ROS_INFO("start spinning");
  while (ros::ok()) {
    ros::spinOnce(); // check for incoming messages
    rate.sleep();
  }
  
  return EXIT_SUCCESS;
}
