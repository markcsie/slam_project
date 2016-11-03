#include <ros/ros.h>
#include <ros/package.h>
#include "../include/data_reader.h"

vector<measure> robot_measurement;
vector<odometry> robot_odometry;
vector<groundtruth> robot_groundtruth;
vector<int> subject;
vector<landmark> landmark_groundtruth;

int k=0; 
//robot_measurement robot_odometry robot_groundtruth subject landmark_groundtruth
void readData(){
  //read data
  //vector<measure> robot_measurment;
  string path = ros::package::getPath("slam_project");
  ifstream file1(path+"/data/newRobot1_Measurement.txt");
  ifstream file1_2(path+ "/data/newRobot1_Measurement.txt");
  int line_count1 = 0;
  string line1;
  double m_time, m_range, m_bearing;
  int m_subject;
  measure m_cur;
  while (getline(file1, line1))
    ++line_count1;
  for (int i = 0; i < line_count1; i++) {
    file1_2 >> m_time >> m_subject >> m_range>>m_bearing;
    m_cur.id = i;
    m_cur.time = m_time;
    m_cur.subject = m_subject;
    m_cur.range = m_range;
    m_cur.bearing = m_bearing;
    robot_measurement.push_back(m_cur);
  }
  file1.close();
  file1_2.close();

  //odometry
  //vector<odometry> robot_odometry;
  ifstream file2(path+"/data/newRobot1_Odometry.txt");
  ifstream file2_2(path+"/data/newRobot1_Odometry.txt");
  int line_count2 = 0;
  string line2;
  double o_time, o_forward_velocity, o_angular_velocity;
  odometry o_cur;
  while (getline(file2, line2))
    ++line_count2;
  for (int i = 0; i < line_count2; i++) {
    file2_2 >> o_time >> o_forward_velocity>>o_angular_velocity;
    o_cur.id = i;
    o_cur.time = o_time;
    o_cur.forward_velocity = o_forward_velocity;
    o_cur.angular_veolocity = o_angular_velocity;
    robot_odometry.push_back(o_cur);
  }
  file2.close();
  file2_2.close();

  //groundtruth
  //vector<groundtruth> robot_groundtruth;
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

  //barcode
  //vector<int> subject;
  ifstream file4(path+"/data/Barcodes.dat");
  ifstream file4_2(path+"/data/Barcodes.dat");
  subject.push_back(0);
  int line_count4 = 0;
  string line4;
  int subject_index, barcode_index;
  while (getline(file4, line4))
    ++line_count4;
  for (int i = 0; i < line_count4; i++) {
    file4_2 >> subject_index>>barcode_index;
    subject.push_back(barcode_index);
  }
  file4.close();
  file4_2.close();

  //Landmark_Groundtruth
  //vector<landmark> landmark_groundtruth;
  ifstream file5(path+"/data/Landmark_Groundtruth.dat");
  ifstream file5_2(path+"/data/Landmark_Groundtruth.dat");
  string line5;
  int line5_count;
  double l_x, l_y, l_xstd_dev, l_ystd_dev;
  int l_subject;
  landmark l_cur;
  while (getline(file5, line5))
    ++line5_count;
  for (int i = 0; i < line5_count; i++) {
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

slam_project::Robot_GroundTruth sendMsg_GroundTruth(int i){
    slam_project::Robot_GroundTruth msg;
 
    if (i<robot_groundtruth.size()){
      msg.time = robot_groundtruth[i].time;
      msg.x = robot_groundtruth[i].x;
      msg.y = robot_groundtruth[i].y;
      msg.orientation = robot_groundtruth[i].orientation;
      i++;  
    }
    return msg;
}

slam_project::Robot_Odometry sendMsg_Odometry(int j){

    slam_project::Robot_Odometry msg_odometry;
    if (j</*robot_odometry.size()*/7000){
  
      msg_odometry.time = robot_odometry[j].time;
      msg_odometry.forward_velocity = robot_odometry[j].forward_velocity;
      msg_odometry.angular_velocity = robot_odometry[j].angular_veolocity;

      int count = 0;
      vector<int> msg_subject;
      vector<double> msg_range;
      vector<double> msg_bearing;
      //only robot1's data is transmitted.
      if (robot_measurement[k].time == msg_odometry.time){
      
        while(robot_measurement[k].time == msg_odometry.time){
          if (robot_measurement[k].subject != subject[2] && 
             robot_measurement[k].subject != subject[3] &&
             robot_measurement[k].subject != subject[4] &&
             robot_measurement[k].subject != subject[5]){
              
            msg_subject.push_back(robot_measurement[k].subject);
            msg_range.push_back(robot_measurement[k].range);
            msg_bearing.push_back(robot_measurement[k].bearing);
            count++;
          }
          k++;
        }  

      }

      if (count){
        msg_odometry.subject.resize(count);
        msg_odometry.range.resize(count);
        msg_odometry.bearing.resize(count);
        for (int i=0; i<count; i++){
          msg_odometry.subject[i] = msg_subject[i];
          msg_odometry.range[i] = msg_range[i];
          msg_odometry.bearing[i] = msg_bearing[i];
        }

      }
      msg_odometry.num = count;



      cout<<"j: "<<j<<" time: "<<msg_odometry.time<<" "<<
          "num: "<<msg_odometry.num<<endl;
      /*dataPublisher2.publish(msg_odometry);*/
    }
    return msg_odometry;
}

bool add(slam_project::requestBarcode::Request &req,
         slam_project::requestBarcode::Response &res){
  if (req.a!=1) return false;
  int len=subject.size()-1;
  res.barcode.resize(len+1);
  res.barcode[0] = len;
  for (int i=1; i<=len; i++){
    res.barcode[i] = subject[i];
    ROS_INFO("sending back response: %d", (int)res.barcode[i]);  
  }
  ROS_INFO("request barcode data");
  return true;
}

int main(int argc, char **argv) {
  readData();
  ros::init(argc, argv, "data_reader");  //node name "data_reader"
  ros::NodeHandle node;

  ros::Publisher dataPublisher2 = node.advertise<slam_project::Robot_Odometry>("/publishMsg2", 1000);
  ros::Publisher dataPublisher3 = node.advertise<slam_project::Robot_GroundTruth>("/publishMsg3", 1000);
  ros::ServiceServer service = node.advertiseService("requestData", add);
  ROS_INFO("Ready to send barcode data.");
//  ros::spin();


  ros::Rate rate(100);
  ROS_INFO("start spinning");
  int i=0,j=0;

  cout<<"robot odometry size: "<<robot_odometry.size()<<endl;
  while (ros::ok()) {
    //dataPublisher3.publish(sendMsg_GroundTruth(i));
    dataPublisher2.publish(sendMsg_Odometry(j));
    j++;
    ros::spinOnce(); // check for incoming messages
    rate.sleep();
    cout<<robot_odometry.size()<<endl;
  }

  return EXIT_SUCCESS;
}
