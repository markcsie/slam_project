#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <ros/package.h>
#include "../include/data_reader.h"
#include <cmath>
vector<groundtruth> robot_groundtruth;
vector<groundtruth> new_robot_groundtruth;
vector<landmark> landmark_groundtruth;
groundtruth new_g;
int k = 0;
geometry_msgs::Point p;
ros::Publisher marker_pub;
ros::Publisher marker_pub2;
ros::Publisher marker_pub3;
ros::Publisher marker_pub4;
ros::Publisher marker_pub5;
visualization_msgs::Marker points, line_strip, line_list, arrow, points2, points3, points4, marker;

void readGroundTruth()
{
  string path = ros::package::getPath("slam_project");

  ifstream file3(path + "/data/newRobot1_Groundtruth.txt");
  ifstream file3_2(path + "/data/newRobot1_Groundtruth.txt");
  int line_count3 = 0;
  string line3;
  double g_time, g_x, g_y, g_orientation;
  groundtruth g_cur;
  while (getline(file3, line3))
    ++line_count3;
  for (int i = 0; i < line_count3; i++)
  {
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

void readlandmarkGroundTruth()
{
  string path = ros::package::getPath("slam_project");

  ifstream file5(path + "/data/Landmark_Groundtruth.dat");
  ifstream file5_2(path + "/data/Landmark_Groundtruth.dat");
  string line5;
  int line5_count;
  double l_x, l_y, l_xstd_dev, l_ystd_dev;
  int l_subject;
  landmark l_cur;
  while (getline(file5, line5))
    ++line5_count;
  for (int i = 0; i < line5_count; i++)
  {
    file5_2 >> l_subject >> l_x >> l_y >> l_xstd_dev>>l_ystd_dev;
    l_cur.subject = l_subject;
    l_cur.x = l_x;
    l_cur.y = l_y;
    l_cur.xstd_dev = l_xstd_dev;
    l_cur.ystd_dev = l_ystd_dev;
    landmark_groundtruth.push_back(l_cur);
  }
  file5.close();
  file5_2.close();
}

void publishMsg_callback(const slam_project::Robot_GroundTruth& subMsg)
{
  new_g.x = subMsg.x;
  new_g.y = subMsg.y;
  cout << new_g.x << endl;
  new_robot_groundtruth.push_back(new_g);
  cout << "*******" << subMsg.num << endl;
  int len = subMsg.num;
  if (k < 7000 /*new_robot_groundtruth.size()*/)
  {
    p.x = robot_groundtruth[k].x;
    p.y = robot_groundtruth[k].y;
    points.points.push_back(p);

    p.x = new_robot_groundtruth[k].x;
    p.y = new_robot_groundtruth[k].y;
    points2.points.push_back(p);
    cout << "**************" << p.x << " " << p.y << endl;
    k++;


    for (int i = 0; i < len; i++)
    {
      p.x = subMsg.landmark_x[i];
      p.y = subMsg.landmark_y[i];
      points4.points.push_back(p);
      cout << "**************" << p.x << " " << p.y << endl;
    }

    marker_pub.publish(points);
    marker_pub2.publish(points2);
    marker_pub4.publish(points4);
  }


}

int main(int argc, char** argv)
{
  readGroundTruth();
  readlandmarkGroundTruth();
  ros::init(argc, argv, "points_and_lines");

  ros::NodeHandle n;
  marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);
  marker_pub2 = n.advertise<visualization_msgs::Marker>("visualization_marker2", 10);
  marker_pub3 = n.advertise<visualization_msgs::Marker>("visualization_marker3", 10);
  marker_pub4 = n.advertise<visualization_msgs::Marker>("visualization_marker4", 10);
  //ellipse
  marker_pub5 = n.advertise<visualization_msgs::Marker>("visualization_marker5", 10);
  cout << "aaa" << endl;
  ros::Subscriber subscriber = n.subscribe("/publishMsg4", 1000, publishMsg_callback);
  cout << new_robot_groundtruth.size() << endl;

  ros::Rate r(100);

  float f = 0.0;

  //  visualization_msgs::Marker points, line_strip, line_list, arrow, points2;
  marker.header.frame_id = points4.header.frame_id = points3.header.frame_id = points2.header.frame_id = points.header.frame_id = line_strip.header.frame_id = line_list.header.frame_id = "map";
  marker.header.stamp = points4.header.stamp = points3.header.stamp = points2.header.stamp = points.header.stamp = line_strip.header.stamp = line_list.header.stamp = ros::Time::now();
  marker.ns = points4.ns = points3.ns = points2.ns = points.ns = line_strip.ns = line_list.ns = "points_and_lines";
  points4.action = points3.action = points2.action = points.action = line_strip.action = line_list.action = visualization_msgs::Marker::ADD;
  points4.pose.orientation.w = points3.pose.orientation.w = points2.pose.orientation.w = points.pose.orientation.w = line_strip.pose.orientation.w = line_list.pose.orientation.w = 1.0;

  points4.type = points3.type = points2.type = points.type = visualization_msgs::Marker::POINTS;
  marker.type = visualization_msgs::Marker::CYLINDER;
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

  points3.scale.x = 0.1;
  points3.scale.y = 0.1;

  points3.color.b = 1.0f;
  points3.color.a = 1.0;

  points4.scale.x = 0.1;
  points4.scale.y = 0.1;

  points4.color.b = 0.5f;
  points4.color.r = 0.5;

  //draw ellipse 
  marker.pose.position.x = 0;
  marker.pose.position.y = 0;
  marker.pose.position.z = 0;

  double t = M_PI/3;
  marker.pose.orientation.x = std::cos(t/2);
  marker.pose.orientation.y = std::sin(t/2);
  marker.pose.orientation.z = 0;
  marker.pose.orientation.w = 0;

  // Set the scale of the marker -- 1x1x1 here means 1m on a side
  marker.scale.x = 2.0;
  marker.scale.y = 1.0;
  marker.scale.z = 0;
  
  marker.color.r = 0.0f;
  marker.color.g = 1.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0;

  for (int i = 0; i < landmark_groundtruth.size(); i++)
  {
    p.x = landmark_groundtruth[i].x;
    p.y = landmark_groundtruth[i].y;
    points3.points.push_back(p);

  }


  cout<<M_PI<<endl;
  cout<<std::atan(1)<<endl;
  while (ros::ok())
  {
    marker_pub3.publish(points3);
    marker_pub5.publish(marker);
    ros::spinOnce();
    r.sleep();
  }
}