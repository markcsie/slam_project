#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <ros/package.h>
#include "../include/data_reader.h"
#include <cmath>

#include <Eigen/Eigenvalues> 

/*
points: robot groundtruth
points2: slam robot groundtruth
points3: landmark groundtruth
points4: slam landmark groundtruth
ellipse: particle ellipse
*/

vector<groundtruth> robot_groundtruth;
vector<landmark> landmark_groundtruth;
int k = 0;
geometry_msgs::Point p;
ros::Publisher marker_pub;
ros::Publisher marker_pub2;
ros::Publisher marker_pub3;
ros::Publisher marker_pub4;
ros::Publisher marker_pub5;
visualization_msgs::Marker points, line_strip, line_list, arrow, points2, points3, points4;
visualization_msgs::MarkerArray ellipse;

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
  if (k < 7000 /*new_robot_groundtruth.size()*/)
  {
    k++;
    p.x = robot_groundtruth[k].x;
    p.y = robot_groundtruth[k].y;
    points.points.clear();     
    points.points.push_back(p);
    points.id = k;
    marker_pub.publish(points);

//    cout << "ggg ************** robot_groundtruth[k] x y " << robot_groundtruth[k].x << " " << robot_groundtruth[k].y << endl;
    if (std::isnan(subMsg.x) || std::isnan(subMsg.y))
    {
      return;
    }

    p.x = subMsg.x;
    p.y = subMsg.y;
    points2.points.clear();
    points2.points.push_back(p);
    points2.id = k;
    marker_pub2.publish(points2);
    cout << "ggg ************** subMsg x y " << subMsg.x << " " << subMsg.y << endl;

    int len = subMsg.num;
    cout << "*******" << subMsg.num << endl;
    int valid_len = 0, valid_i=0;
    for (int i=0; i<len; i++)
      if (!std::isnan(subMsg.landmark_x[i] && !std::isnan(subMsg.landmark_y[i]) ))
        valid_len ++;

    ellipse.markers.resize(valid_len);
    for (int i = 0; i < len; i++)
    {
      if (std::isnan(subMsg.landmark_x[i]) || std::isnan(subMsg.landmark_y[i]))
      {
        continue;
      }

      p.x = subMsg.landmark_x[i];
      p.y = subMsg.landmark_y[i];
      points4.points.clear();
      points4.points.push_back(p);
      points4.id = k*20+i;
//      marker_pub4.publish(points4);
      cout << "ggg ************** subMsg.landmark [i] " << subMsg.landmark_x[i] << " " << subMsg.landmark_y[i] << endl;

      Eigen::MatrixXd cov(2, 2);
      cov(0, 0) = subMsg.landmark_cov[i].data[0];
      cov(0, 1) = subMsg.landmark_cov[i].data[1];
      cov(1, 0) = subMsg.landmark_cov[i].data[2];
      cov(1, 1) = subMsg.landmark_cov[i].data[3];
   //TODO   std::cout << "ggg cov " << std::endl;
   //TODO   std::cout << cov << std::endl;

      Eigen::EigenSolver<Eigen::MatrixXd> es(cov);

   //TODO   std::cout << "ggg" << std::endl;
      Eigen::VectorXd axis1 = es.eigenvalues().real()[0] * es.eigenvectors().real().col(0);
      Eigen::VectorXd axis2 = es.eigenvalues().real()[1] * es.eigenvectors().real().col(1);
   //TODO   std::cout << "ggg" << std::endl;
      // TODO: draw an ellipse with these two axes

      ellipse.markers[valid_i].header.frame_id = "map";
      ellipse.markers[valid_i].header.stamp = ros::Time::now();
      ellipse.markers[valid_i].ns = "points_and_lines";
      ellipse.markers[valid_i].action = visualization_msgs::Marker::ADD;
      ellipse.markers[valid_i].type = visualization_msgs::Marker::CYLINDER;

      double angle = std::atan(axis1[1] / axis1[0]);

      ellipse.markers[valid_i].pose.orientation.x = std::cos(angle / 2);
      ellipse.markers[valid_i].pose.orientation.y = std::sin(angle / 2);
      ellipse.markers[valid_i].pose.orientation.z = 0;
      ellipse.markers[valid_i].pose.orientation.w = 0;

      //draw ellipse 
      ellipse.markers[valid_i].pose.position.x = p.x;
      ellipse.markers[valid_i].pose.position.y = p.y;
      ellipse.markers[valid_i].pose.position.z = 0;


      // Set the scale of the marker -- 1x1x1 here means 1m on a side
      ellipse.markers[valid_i].scale.x = 1000*axis1.norm();  //TODO 500
      ellipse.markers[valid_i].scale.y = 1000*axis2.norm();  //TODO 500
      ellipse.markers[valid_i].scale.z = 0;

      /*cout<<"valid id: "<<valid_i<<"lenght of axis1: "<<axis1.norm()<<endl;
      cout<<"valid id: "<<valid_i<<"lenght of axis2: "<<axis2.norm()<<endl;*/
      ellipse.markers[valid_i].color.r = 0.0f;
      ellipse.markers[valid_i].color.g = 1.0f;
      ellipse.markers[valid_i].color.b = 0.0f;
      ellipse.markers[valid_i].color.a = 1.0;
      
      ellipse.markers[valid_i].id = valid_i; //must add id, or there will be only one marker
      valid_i++;

    }


    marker_pub4.publish(points4);
    marker_pub5.publish(ellipse);
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
  marker_pub5 = n.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 10);
  cout << "aaa" << endl;
  ros::Subscriber subscriber = n.subscribe("/publishMsg4", 1000, publishMsg_callback);

  ros::Rate r(10);

  float f = 0.0;

  //  visualization_msgs::Marker points, line_strip, line_list, arrow, points2;
  points4.header.frame_id = points3.header.frame_id = points2.header.frame_id = points.header.frame_id = line_strip.header.frame_id = line_list.header.frame_id = "map";
  points4.header.stamp = points3.header.stamp = points2.header.stamp = points.header.stamp = line_strip.header.stamp = line_list.header.stamp = ros::Time::now();
  points4.ns = points3.ns = points2.ns = points.ns = line_strip.ns = line_list.ns = "points_and_lines";
  points4.action = points3.action = points2.action = points.action = line_strip.action = line_list.action = visualization_msgs::Marker::ADD;
  points4.pose.orientation.w = points3.pose.orientation.w = points2.pose.orientation.w = points.pose.orientation.w = line_strip.pose.orientation.w = line_list.pose.orientation.w = 1.0;

  points4.type = points3.type = points2.type = points.type = visualization_msgs::Marker::POINTS;
  
  //lifetime
  points.lifetime = ros::Duration();  // A value of ros::Duration() means never to auto-delete
  points2.lifetime = ros::Duration();

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

  points4.color.r = 0.8f;
  points4.color.a = 0.5;


  for (int i = 0; i < landmark_groundtruth.size(); i++)
  {
    p.x = landmark_groundtruth[i].x;
    p.y = landmark_groundtruth[i].y;
    points3.points.push_back(p);

  }


  cout << M_PI << endl;
  cout << std::atan(1) << endl;
  while (ros::ok())
  {
    marker_pub3.publish(points3);
    ros::spinOnce();
    r.sleep();
  }
}