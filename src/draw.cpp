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
geometry_msgs::Point p;
ros::Publisher marker_pub3;
ros::Publisher marker_pub4;
ros::Publisher marker_pub5;
ros::Publisher marker_pub_path;
ros::Publisher marker_pub_slam_path;
visualization_msgs::Marker points3, points4;
visualization_msgs::MarkerArray ellipse;
visualization_msgs::MarkerArray multi_path;  //groundtruth
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

void publishMsg_callback(const slam_project::Robot_Path_Map& subMsg)
{
  
    k++;
    int robot_num = subMsg.rx.size();

    //groundtruth
    for (int i=0; i<robot_num; i++){
      p.x = subMsg.rx[i];
      p.y = subMsg.ry[i];
      multi_path.markers[i].points.clear();
      multi_path.markers[i].points.push_back(p);
      multi_path.markers[i].id = i*100000+k;
    }
    marker_pub_path.publish(multi_path);     


//    cout << "ggg ************** robot_groundtruth[k] x y " << robot_groundtruth[k].x << " " << robot_groundtruth[k].y << endl;
    if (std::isnan(subMsg.x[0]) || std::isnan(subMsg.y[0]))
    {
      return;
    }


    //slam path
    for (int i=0; i<robot_num; i++){
      p.x = subMsg.x[i];
      p.y = subMsg.y[i];
      multi_slam_path.markers[i].points.clear();
      multi_slam_path.markers[i].points.push_back(p);
      multi_slam_path.markers[i].id = (i+2)*100000+k;
    }
    marker_pub_slam_path.publish(multi_slam_path);     

    //slam landmark and ellipse
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

      double angle = std::atan(axis1[1] / axis1[0]);

      ellipse.markers[valid_i].pose.orientation.x = std::cos(angle / 2);
      ellipse.markers[valid_i].pose.orientation.y = std::sin(angle / 2);

      //draw ellipse 
      ellipse.markers[valid_i].pose.position.x = p.x;
      ellipse.markers[valid_i].pose.position.y = p.y;


      // Set the scale of the marker -- 1x1x1 here means 1m on a side
      ellipse.markers[valid_i].scale.x = axis1.norm();  //TODO 500
      ellipse.markers[valid_i].scale.y = axis2.norm();  //TODO 500

      
      valid_i++;

    }


    marker_pub4.publish(points4);
    //  marker_pub5.publish(ellipse);
  
}

void update(){
  cout<<"update start: "<<endl;
  for (int i=0; i<1; i++){
      cout<<multirobot_groundtruth[i].size()<<endl;
      p.x = multirobot_groundtruth[i][k].x;
      p.y = multirobot_groundtruth[i][k].y;
    
    //  cout<<"p.x: "<<p.x<<endl;
    //  cout<<"p.y: "<<p.y<<endl;
      multi_path.markers[i].points.clear();
    //  cout<<"after p.x"<<endl;
      multi_path.markers[i].points.push_back(p);
    //  cout<<"before id: "<<endl;
      multi_path.markers[i].id = i*100000+k;

    //  cout<<"end"<<endl;
  }
//  cout<<"finish"<<endl;
  marker_pub_path.publish(multi_path);     

}

void update2(){


  for (int i=0; i<1; i++){
      
      //geometry_msgs::Point p;
      p.x = multirobot_groundtruth[i][k].x;
      p.y = multirobot_groundtruth[i][k].y;
    
      points4.points.push_back(p);
      //points4.id = i*100000+k;

  }
  marker_pub4.publish(points4);     



}

void init_marker(){
  float f = 0.0;
  points4.header.frame_id = points3.header.frame_id = "map";
  points4.header.stamp = points3.header.stamp = ros::Time::now();
  points4.ns = points3.ns  = "points_and_lines";
  points4.action = points3.action = visualization_msgs::Marker::ADD;
  points4.pose.orientation.w = points3.pose.orientation.w = 1.0;

  points4.type = points3.type = visualization_msgs::Marker::POINTS;
  

  points3.scale.x = 0.1;
  points3.scale.y = 0.1;

  points3.color.b = 1.0f;
  points3.color.a = 1.0;

  points4.scale.x = 0.1;
  points4.scale.y = 0.1;

  points4.color.r = 0.8f;
  points4.color.a = 0.5;

  int robot_num = 1;
  multi_path.markers.resize(robot_num);
  multi_slam_path.markers.resize(robot_num);
  for (int i=0; i<robot_num; i++){
    //groundtruth
    multi_path.markers[i].header.frame_id = "map";
    multi_path.markers[i].header.stamp = ros::Time::now();
    multi_path.markers[i].ns = "points_and_lines";
    multi_path.markers[i].action = visualization_msgs::Marker::ADD;
    multi_path.markers[i].type = visualization_msgs::Marker::POINTS;
    multi_path.markers[i].lifetime = ros::Duration();
     // A value of ros::Duration() means never to auto-delete

    multi_path.markers[i].scale.x = 0.05;
    multi_path.markers[i].scale.y = 0.05;
    multi_path.markers[i].color.r = 0.8f;
    multi_path.markers[i].color.a = 0.5;

    //calculated path from slam
    multi_slam_path.markers[i].header.frame_id = "map";
    multi_slam_path.markers[i].header.stamp = ros::Time::now();
    multi_slam_path.markers[i].ns = "points_and_lines";
    multi_slam_path.markers[i].action = visualization_msgs::Marker::ADD;
    multi_slam_path.markers[i].type = visualization_msgs::Marker::POINTS;
    multi_slam_path.markers[i].lifetime = ros::Duration();

    multi_slam_path.markers[i].scale.x = 0.05;
    multi_slam_path.markers[i].scale.y = 0.05;
    multi_slam_path.markers[i].color.g = 0.8f;
    multi_slam_path.markers[i].color.a = 0.5;
  }

  //15 is the number of landmarks

  ellipse.markers.resize(15);
  for (int i=0; i<15; i++){
    int valid_i = i;
    ellipse.markers[valid_i].header.frame_id = "map";
    ellipse.markers[valid_i].header.stamp = ros::Time::now();
    ellipse.markers[valid_i].ns = "points_and_lines";
    ellipse.markers[valid_i].action = visualization_msgs::Marker::ADD;
    ellipse.markers[valid_i].type = visualization_msgs::Marker::CYLINDER;
    ellipse.markers[valid_i].lifetime = ros::Duration();
    
    ellipse.markers[valid_i].pose.orientation.z = 0;
    ellipse.markers[valid_i].pose.orientation.w = 0;

    ellipse.markers[valid_i].pose.position.z = 0;


    ellipse.markers[valid_i].scale.z = 0;

    ellipse.markers[valid_i].color.r = 0.0f;
    ellipse.markers[valid_i].color.g = 1.0f;
    ellipse.markers[valid_i].color.b = 0.0f;
    ellipse.markers[valid_i].color.a = 1.0;
      
    ellipse.markers[valid_i].id = valid_i; //must add id, or there will be only one marker
  
  }
}

int main(int argc, char** argv)
{
  cout<<"***********start********* "<<endl;
  readlandmarkGroundTruth();
  ros::init(argc, argv, "points_and_lines");
  cout<<"finish readlandmarkGroundTruth"<<endl;
  ros::NodeHandle n;
  //groundtruth landmark
  marker_pub3 = n.advertise<visualization_msgs::Marker>("visualization_marker3", 100000);
  //slam landmark
  marker_pub4 = n.advertise<visualization_msgs::Marker>("visualization_marker4", 100000);
  //ellipse
  marker_pub5 = n.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 100000);
  //multi_path groundtruth
  marker_pub_path = n.advertise<visualization_msgs::MarkerArray>("groundtruth_path", 100000);
  //slam path
  marker_pub_slam_path = n.advertise<visualization_msgs::MarkerArray>("slam_path", 100000);
    
  cout<<"before init marker"<<endl; 
  init_marker();
  
  cout << "******ros start*********" << endl;
  //ros::Subscriber subscriber = n.subscribe("/publishMsg4", 100000, publishMsg_callback);

/*  
  for (int i = 0; i < landmark_groundtruth.size(); i++)
  {
    p.x = landmark_groundtruth[i].x;
    p.y = landmark_groundtruth[i].y;
    points3.points.push_back(p);
  }
  
  while (marker_pub3.getNumSubscribers() == 0) // TODO: Give time to Rviz to be fully started
  {
  
  }
  marker_pub3.publish(points3);*/
  

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
    update2();
    k++;
    cout<<k<<endl;
    //ros::spinOnce();
    r.sleep();
  }
}