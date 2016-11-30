#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <ros/package.h>
#include "../include/data_reader.h"
#include <cmath>

#include <Eigen/Eigenvalues> 

/*
points3: landmark groundtruth
points4: slam landmark groundtruth
ellipse: particle ellipse
multi_path: groundtruth
multi_slam_path: result path from slam
*/

vector<landmark> landmark_groundtruth;
int k = 0;
//geometry_msgs::Point p;
ros::Publisher marker_pub3;
ros::Publisher marker_pub4;
ros::Publisher marker_pub5;
ros::Publisher marker_pub_path;
ros::Publisher marker_pub_slam_path;
visualization_msgs::Marker points3, points4;
visualization_msgs::MarkerArray ellipse;
//visualization_msgs::MarkerArray multi_path;  //groundtruth
visualization_msgs::MarkerArray multi_slam_path; //result from slam
std::vector<std::vector<groundtruth> > multirobot_groundtruth;


void readGroundtruth(const int index, const string path_groundtruth)
{
  ifstream file3(path_groundtruth);
  ifstream file3_2(path_groundtruth);
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
    multirobot_groundtruth[index].push_back(g_cur);
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
  //TODO
/*  while (getline(file5, line5))
    ++line5_count;*/
  line5_count=15;
  cout<<"line5_count: "<<line5_count<<endl;
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

void init_marker(){
 

}

void update(){

  visualization_msgs::MarkerArray multi_path;  //groundtruth

  float f = 0.0;
 
  int robot_num = 1;
  multi_path.markers.resize(robot_num);
  for (int i=0; i<robot_num; i++){
    //groundtruth
    multi_path.markers[i].header.frame_id = "map";
    multi_path.markers[i].header.stamp = ros::Time::now();
    multi_path.markers[i].ns = "points_and_lines";
    multi_path.markers[i].action = visualization_msgs::Marker::ADD;
    multi_path.markers[i].type = visualization_msgs::Marker::POINTS;
   // multi_path.markers[i].lifetime = ros::Duration(0);
     // A value of ros::Duration() means never to auto-delete

    multi_path.markers[i].scale.x = 0.05;
    multi_path.markers[i].scale.y = 0.05;
    multi_path.markers[i].color.r = 0.8f;
    multi_path.markers[i].color.a = 0.5;
    multi_path.markers[i].id = i;


  }
  cout<<"update start: "<<endl;
  for (int i=0; i<1; i++){
    for (int j=0; j<k; j++){
      geometry_msgs::Point p;
      p.x = multirobot_groundtruth[i][j].x;
      p.y = multirobot_groundtruth[i][j].y;
    
      cout<<"p.x: "<<p.x<<endl;
      cout<<"p.y: "<<p.y<<endl;
//      multi_path.markers[i].points.clear();
      cout<<"after p.x"<<endl;
      multi_path.markers[i].points.push_back(p);
      cout<<"before id: "<<endl;
//      multi_path.markers[i].id = i*100000+k;

    }
  }
  cout<<"finish"<<endl;
  cout<<multi_path.markers[0].points.size()<<endl;
  marker_pub_path.publish(multi_path);     

}





int main(int argc, char** argv)
{
  cout<<"***********start********* "<<endl;
  readlandmarkGroundTruth();
  ros::init(argc, argv, "points_and_lines");
  ros::NodeHandle n;
  //multi_path groundtruth
  marker_pub_path = n.advertise<visualization_msgs::MarkerArray>("groundtruth_path", 10000);

//  init_marker();


  multirobot_groundtruth.resize(2);
  int robot_num = 2;

  cout<<"read data"<<endl;
  string path = ros::package::getPath("slam_project");
  for (int i = 1; i <= robot_num; i++)
  {
    stringstream ss;
    ss << i;
    string str = ss.str();
    string path_groundtruth = path + "/data/newRobot" + str + "_Groundtruth.txt";
    readGroundtruth(i - 1, path_groundtruth);
  }
  ros::Rate r(100);
  cout<<"begin update"<<endl;



  while (ros::ok())
  {
    update();
    k++;
    cout<<k<<endl;
    //ros::spinOnce();
    r.sleep();
  }
}