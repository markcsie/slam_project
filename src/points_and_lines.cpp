#include <ros/ros.h>
#include <nav_msgs/Path.h>
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


ros::Publisher pub_groundtruth_path1;
ros::Publisher pub_groundtruth_path2;
ros::Publisher pub_groundtruth_path3;
ros::Publisher pub_groundtruth_path4;
ros::Publisher pub_groundtruth_path5;

ros::Publisher pub_slam_path1;
ros::Publisher pub_slam_path2;
ros::Publisher pub_slam_path3;
ros::Publisher pub_slam_path4;
ros::Publisher pub_slam_path5;

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

vector<vector<groundtruth> > multi_groundtruth;
vector<vector<geometry_msgs::PoseStamped>> poses;
vector<vector<geometry_msgs::PoseStamped>> slam_poses;
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

//init mark of landmark 
void init_marker1(){
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
  

  ellipse.markers.resize(15);
  for (int i=0; i<15; i++){
    int valid_i = i;
    ellipse.markers[valid_i].header.frame_id = "map";
    ellipse.markers[valid_i].header.stamp = ros::Time::now();
    ellipse.markers[valid_i].ns = "points_and_lines";
    ellipse.markers[valid_i].action = visualization_msgs::Marker::ADD;
    ellipse.markers[valid_i].type = visualization_msgs::Marker::CYLINDER;

    
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

/*
1 5
2 14
3 41
4 32
5 23
init marker of slam_path, groundtruth path
*/

void init_marker2(int robot_num, int path_num, vector<int> path_id){
  //multi_path: groundtruth, multi_slam_path: slam path
  multi_path.markers.resize(robot_num);
  multi_slam_path.markers.resize(9);
  
  for (int i=0; i<9; i++){
    multi_slam_path.markers[i].header.frame_id = "map";
    multi_slam_path.markers[i].header.stamp = ros::Time::now();
    multi_slam_path.markers[i].ns = "points_and_lines";
    multi_slam_path.markers[i].action = visualization_msgs::Marker::ADD;
    multi_slam_path.markers[i].type = visualization_msgs::Marker::POINTS;
    multi_slam_path.markers[i].lifetime = ros::Duration();

    multi_slam_path.markers[i].scale.x = 0.05;
    multi_slam_path.markers[i].scale.y = 0.05;
  }

  for (int i=0; i<path_num; i++){


    int j=path_id[i];
//    cout<<"j: "<<j<<endl;
    if (j==5){
      multi_slam_path.markers[0].color.r = 0.3f;  
      multi_slam_path.markers[0].color.a = 1.0;
    }else if (j==14 || j==-14){
      if (j>0){
        multi_slam_path.markers[1].color.g = 0.3f;
        multi_slam_path.markers[1].color.a = 1.0;
      }else{
        multi_slam_path.markers[2].color.g = 0.3f;
        multi_slam_path.markers[2].color.a = 1.0;
      }
    }else if (j==41 || j==-41){
      if (j>0){
        multi_slam_path.markers[3].color.b = 0.3f;
        multi_slam_path.markers[3].color.a = 1.0;
      }else{
        multi_slam_path.markers[4].color.b = 0.3f;
        multi_slam_path.markers[4].color.a = 1.0;
      }
    }else if (j==32 || j==-32){
      if (j>0){
        multi_slam_path.markers[5].color.r = 0.3f; 
        multi_slam_path.markers[5].color.g = 0.3f;
        multi_slam_path.markers[5].color.a = 1.0;
      }else{
        multi_slam_path.markers[6].color.r = 0.3f; 
        multi_slam_path.markers[6].color.g = 0.3f;
        multi_slam_path.markers[6].color.a = 1.0;
      }
     }else{
      if (j>0){    
        multi_slam_path.markers[7].color.r = 0.3f; 
        multi_slam_path.markers[7].color.b = 0.3f;
        multi_slam_path.markers[7].color.a = 1.0;
      }else{
        multi_slam_path.markers[8].color.r = 0.3f; 
        multi_slam_path.markers[8].color.b = 0.3f;
        multi_slam_path.markers[8].color.a = 1.0;
      }
    }    

  }


  //15 is the number of landmarks

  
}


void update_groundtruth(const slam_project::Robot_Path_Map& subMsg){

  cout<<"update_groundtruth"<<endl;
  nav_msgs::Path groundtruth_path1;
  nav_msgs::Path groundtruth_path2;
  nav_msgs::Path groundtruth_path3;
  nav_msgs::Path groundtruth_path4;
  nav_msgs::Path groundtruth_path5;

  groundtruth_path1.header.frame_id = "map";
  groundtruth_path1.header.stamp = ros::Time::now();
  groundtruth_path2.header.frame_id = "map";
  groundtruth_path2.header.stamp = ros::Time::now();
  groundtruth_path3.header.frame_id = "map";
  groundtruth_path3.header.stamp = ros::Time::now();
  groundtruth_path4.header.frame_id = "map";
  groundtruth_path4.header.stamp = ros::Time::now();
  groundtruth_path5.header.frame_id = "map";
  groundtruth_path5.header.stamp = ros::Time::now();
      

  for (int i=0; i<subMsg.rx.size(); i++){
    cout<<"k: "<<k<<endl;
    geometry_msgs::PoseStamped cur_pose;
    cur_pose.pose.position.x = subMsg.rx[i];
    cur_pose.pose.position.y = subMsg.ry[i];
    cout<<"x: "<<cur_pose.pose.position.x<<endl;
    poses[i].push_back(cur_pose);

/*    std::vector<geometry_msgs::PoseStamped> poses(k);
    for (int j=0; j<k; j++){
      cout<<j<<endl;
      cout<<multi_groundtruth[i].size()<<endl;
      poses.at(j).pose.position.x = multi_groundtruth[i][j].x;
      poses.at(j).pose.position.y = multi_groundtruth[i][j].y;
    }
*/

    if (i==0)
      groundtruth_path1.poses = poses[i];
    else if (i==1)
      groundtruth_path2.poses = poses[i];
    else if (i==2)
      groundtruth_path3.poses = poses[i];
    else if (i==3)
      groundtruth_path4.poses = poses[i];
    else 
      groundtruth_path5.poses = poses[i];

  }
  pub_groundtruth_path1.publish(groundtruth_path1);
  pub_groundtruth_path2.publish(groundtruth_path2);
  pub_groundtruth_path3.publish(groundtruth_path3);
  pub_groundtruth_path4.publish(groundtruth_path4);
  pub_groundtruth_path5.publish(groundtruth_path5);
  cout<<"end publish"<<endl;
}


void update_slampath(const slam_project::Robot_Path_Map& subMsg){
  nav_msgs::Path slam_path1;
  nav_msgs::Path slam_path2;
  nav_msgs::Path slam_path3;
  nav_msgs::Path slam_path4;
  nav_msgs::Path slam_path5;

  slam_path1.header.frame_id = "map";
  slam_path1.header.stamp = ros::Time::now();
  slam_path2.header.frame_id = "map";
  slam_path2.header.stamp = ros::Time::now();
  slam_path3.header.frame_id = "map";
  slam_path3.header.stamp = ros::Time::now();
  slam_path4.header.frame_id = "map";
  slam_path4.header.stamp = ros::Time::now();
  slam_path5.header.frame_id = "map";
  slam_path5.header.stamp = ros::Time::now();
      

  for (int i=0; i<subMsg.rx.size(); i++){
    geometry_msgs::PoseStamped cur_pose;
    cur_pose.pose.position.x = subMsg.x[i];
    cur_pose.pose.position.y = subMsg.y[i];
    cout<<"i: "<<cur_pose.pose.position.x<<endl;
    slam_poses[i].push_back(cur_pose);

    if (i==0)
      slam_path1.poses = slam_poses[i];
    else if (i==1)
      slam_path2.poses = slam_poses[i];
    else if (i==2)
      slam_path3.poses = slam_poses[i];
    else if (i==3)
      slam_path4.poses = slam_poses[i];
    else 
      slam_path5.poses = slam_poses[i];

  }
  pub_slam_path1.publish(slam_path1);
  pub_slam_path2.publish(slam_path2);
  pub_slam_path3.publish(slam_path3);
  pub_slam_path4.publish(slam_path4);
  pub_slam_path5.publish(slam_path5);
}
void publishMsg_callback(const slam_project::Robot_Path_Map& subMsg)
{
//    cout<<"call back start"<<endl;
    k++;
    cout<<k<<endl;
    int robot_num = subMsg.rx.size();
    int path_num = subMsg.robot_id.size();
    for (int i=0; i<robot_num; i++){
      groundtruth cur_g;
      cur_g.x = subMsg.rx[i];
      cur_g.y = subMsg.ry[i];
      multi_groundtruth[i].push_back(cur_g);
    }

    update_groundtruth(subMsg);
    update_slampath(subMsg);
    /*vector<int> rid;
    for (int i=0; i<subMsg.robot_id.size(); i++)
      rid.push_back(subMsg.robot_id[i]);

    init_marker2(robot_num, path_num, rid);
    

    //groundtruth
    for (int i=0; i<robot_num; i++){

      p.x = subMsg.rx[i];
      p.y = subMsg.ry[i];

    //  multi_path.markers[i].points.clear();
      multi_path.markers[i].points.push_back(p);
      multi_path.markers[i].id =i;//(i+9)*100000+k;
    }
    marker_pub_path.publish(multi_path);     


//    cout << "ggg ************** robot_groundtruth[k] x y " << robot_groundtruth[k].x << " " << robot_groundtruth[k].y << endl;
    if (std::isnan(subMsg.x[0]) || std::isnan(subMsg.y[0]))
    {
      return;
    }


//1 5
//2 14
//3 41
//4 32
//5 23


    //slam path
    for (int i=0; i<path_num; i++){
      p.x = subMsg.x[i];
      p.y = subMsg.y[i];
      int j=rid[i];
    //  multi_slam_path.markers[i].id = i;

      if (j==5){
    //    multi_slam_path.markers[0].points.clear();
        multi_slam_path.markers[0].points.push_back(p);
        multi_slam_path.markers[0].id = 10;//*100000+k;
      }else if (j==14){
    //    multi_slam_path.markers[1].points.clear();
        multi_slam_path.markers[1].points.push_back(p);
        multi_slam_path.markers[1].id = 11;//*100000+k;
      }else if (j==-14){
    //    multi_slam_path.markers[2].points.clear();
        multi_slam_path.markers[2].points.push_back(p);
        multi_slam_path.markers[2].id = 12;//*100000+k;
      }else if (j==41){
    //    multi_slam_path.markers[3].points.clear();
        multi_slam_path.markers[3].points.push_back(p);
        multi_slam_path.markers[3].id = 13;//*100000+k;
      }else if (j==-41){
    //    multi_slam_path.markers[4].points.clear();
        multi_slam_path.markers[4].points.push_back(p);
        multi_slam_path.markers[4].id = 14;//*100000+k;
      }else if (j==32){
    //    multi_slam_path.markers[5].points.clear();
        multi_slam_path.markers[5].points.push_back(p);
        multi_slam_path.markers[5].id = 15;//*100000+k;
      }else if (j==-32){
    //    multi_slam_path.markers[6].points.clear();
        multi_slam_path.markers[6].points.push_back(p);
        multi_slam_path.markers[6].id = 16;//*100000+k;
      }else if (j==23){
    //    multi_slam_path.markers[7].points.clear();
        multi_slam_path.markers[7].points.push_back(p);
        multi_slam_path.markers[7].id = 17;//*100000+k;
      }else{
    //    multi_slam_path.markers[8].points.clear();
        multi_slam_path.markers[8].points.push_back(p);
        multi_slam_path.markers[8].id = 18;//*100000+k;
      }
    }
    marker_pub_slam_path.publish(multi_slam_path);     

    //slam landmark and ellipse
    int len = subMsg.num;
//c    cout << "*******" << subMsg.num << endl;
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
//      cout << "ggg ************** subMsg.landmark [i] " << subMsg.landmark_x[i] << " " << subMsg.landmark_y[i] << endl;

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

//    cout<<"call back end"<<endl;
    marker_pub4.publish(points4);
    marker_pub5.publish(ellipse);
  */
}



int main(int argc, char** argv)
{
  cout<<"***********start********* "<<endl;
  readlandmarkGroundTruth();
  ros::init(argc, argv, "points_and_lines");
  cout<<"finish readlandmarkGroundTruth"<<endl;
  ros::NodeHandle n;
 

  pub_groundtruth_path1 = n.advertise<nav_msgs::Path>("groundtruth_path1", 10000);
  pub_groundtruth_path2 = n.advertise<nav_msgs::Path>("groundtruth_path2", 10000);
  pub_groundtruth_path3 = n.advertise<nav_msgs::Path>("groundtruth_path3", 10000);
  pub_groundtruth_path4 = n.advertise<nav_msgs::Path>("groundtruth_path4", 10000);
  pub_groundtruth_path5 = n.advertise<nav_msgs::Path>("groundtruth_path5", 10000);

  pub_slam_path1 = n.advertise<nav_msgs::Path>("slam_path1", 10000);
  pub_slam_path2 = n.advertise<nav_msgs::Path>("slam_path2", 10000);
  pub_slam_path3 = n.advertise<nav_msgs::Path>("slam_path3", 10000);
  pub_slam_path4 = n.advertise<nav_msgs::Path>("slam_path4", 10000);
  pub_slam_path5 = n.advertise<nav_msgs::Path>("slam_path5", 10000);



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
  init_marker1();
  
  ros::Subscriber subscriber = n.subscribe("/publishMsg4", 100000, publishMsg_callback);

  
  for (int i = 0; i < landmark_groundtruth.size(); i++)
  {
    p.x = landmark_groundtruth[i].x;
    p.y = landmark_groundtruth[i].y;
    points3.points.push_back(p);
  }
  
  while (marker_pub3.getNumSubscribers() == 0) // TODO: Give time to Rviz to be fully started
  {
  
  }
  marker_pub3.publish(points3);
  
  string path = ros::package::getPath("slam_project");
  int robot_num = 3;  
  multi_groundtruth.resize(robot_num);
  poses.resize(robot_num);
  slam_poses.resize(robot_num);
  cout << "******ros start*********" << endl;
 
  while (ros::ok())
  {
    ros::spinOnce();
  }
}