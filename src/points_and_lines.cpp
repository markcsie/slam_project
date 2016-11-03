#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <ros/package.h>
#include "../include/data_reader.h"
#include <cmath>
vector<groundtruth> robot_groundtruth;
vector<groundtruth> new_robot_groundtruth;
groundtruth new_g;
int k=0;
geometry_msgs::Point p;
ros::Publisher marker_pub;
ros::Publisher marker_pub2;
 
visualization_msgs::Marker points, line_strip, line_list, arrow, points2;

void readGroundTruth(){
  string path = ros::package::getPath("slam_project");
  
  ifstream file3(path+"/data/newRobot1_Groundtruth.txt");
  ifstream file3_2(path+"/data/newRobot1_Groundtruth.txt");
  int line_count3 = 0;
  string line3;
  double g_time, g_x, g_y, g_orientation;
  groundtruth g_cur;
  while (getline(file3, line3))
    ++line_count3;
  for (int i = 0; i < line_count3; i++) {
    file3_2 >> g_time >> g_x >> g_y>>g_orientation;
    g_cur.id = i;
    g_cur.time = g_time;
    g_cur.x = g_x;
    g_cur.y = g_y;
    g_cur.orientation = g_orientation;
    robot_groundtruth.push_back(g_cur);
  }
  file3.close();
  file3_2.close();

}


void publishMsg_callback(const slam_project::Robot_GroundTruth& subMsg){
  new_g.x = subMsg.x;
  new_g.y = subMsg.y;
  cout<<new_g.x<<endl;
  new_robot_groundtruth.push_back(new_g);

  if (k<7000 /*new_robot_groundtruth.size()*/){
    p.x = robot_groundtruth[k].x;
    p.y = robot_groundtruth[k].y;
    points.points.push_back(p);
      
    p.x = new_robot_groundtruth[k].x;
    p.y = new_robot_groundtruth[k].y;
    points2.points.push_back(p);
    k++;
    marker_pub.publish(points);
    marker_pub2.publish(points2);
  }

 
}

int main( int argc, char** argv )
{
  readGroundTruth();
  ros::init(argc, argv, "points_and_lines");
  ros::NodeHandle n;
  marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);
  marker_pub2 = n.advertise<visualization_msgs::Marker>("visualization_marker2", 10);
  
  cout<<"aaa"<<endl;  
  ros::Subscriber subscriber = n.subscribe("/publishMsg4", 1000, publishMsg_callback);
  cout<<new_robot_groundtruth.size()<<endl;

  ros::Rate r(100);

  float f = 0.0;

//  visualization_msgs::Marker points, line_strip, line_list, arrow, points2;
  points2.header.frame_id = points.header.frame_id = line_strip.header.frame_id = line_list.header.frame_id = "map";
  points2.header.stamp = points.header.stamp = line_strip.header.stamp = line_list.header.stamp = ros::Time::now();
  points2.ns = points.ns = line_strip.ns = line_list.ns = "points_and_lines";
  points2.action = points.action = line_strip.action = line_list.action = visualization_msgs::Marker::ADD;
  points2.pose.orientation.w = points.pose.orientation.w = line_strip.pose.orientation.w = line_list.pose.orientation.w = 1.0;

  points2.type = points.type = visualization_msgs::Marker::POINTS;
 
    // POINTS markers use x and y scale for width/height respectively
  points.scale.x = 0.01;
  points.scale.y = 0.01;

    // Points are green
  points.color.g = 1.0f;
  points.color.a = 1.0;

  points2.scale.x = 0.01;
  points2.scale.y = 0.01;

  points2.color.r = 1.0f;
  points2.color.a = 1.0;

  while (ros::ok())
  {
    ros::spinOnce();
    r.sleep();
  }
}