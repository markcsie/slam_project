#include <ros/ros.h>
#include <ros/package.h>
#include "../include/data_reader.h"

vector<measure> robot_measurement;
vector<odometry> robot_odometry;
vector<groundtruth> robot_groundtruth;
vector<int> subject;
vector<landmark> landmark_groundtruth;


//for multi-robot
vector<vector<measure> > multirobot_measurement;
vector<vector<odometry> > multirobot_odometry;
vector<vector<groundtruth> > multirobot_groundtruth;

int k = 0;
vector<int> kk;
//robot_measurement robot_odometry robot_groundtruth subject landmark_groundtruth

//n is the number of robots
void readMultiData(int n){
  multirobot_groundtruth.resize(n);
  multirobot_odometry.resize(n);
  multirobot_measurement.resize(n);
  string path = ros::package::getPath("slam_project");
  for (int i=1; i<=n; i++){
    stringstream ss; 
    ss << i;
    string str = ss.str();
    string path_measure =  path+ "/data/newRobot" + str + "_Measurement.txt";
    string path_odometry = path+ "/data/newRobot" + str + "_Odometry.txt";
    string path_groundtruth = path+ "/data/newRobot" + str + "_Groundtruth.txt";
    readMeasurement(i-1, path_measure);
    readOdometry(i-1, path_odometry);
    //readGroundtruth(i-1, path_groundtruth);
  }
}


void readMeasurement(int index, string path_measure){
  ifstream file1(path_measure);
  ifstream file1_2(path_measure);
  int line_count1 = 0;
  string line1;
  while (getline(file1, line1))
    ++line_count1;
  double m_time, m_bearing, m_range;
  int m_subject;
  measure m_cur;
  for (int i = 0; i < line_count1; i++)
  {
    file1_2 >> m_time >> m_subject >> m_range>>m_bearing;
    m_cur.id = i;
    m_cur.time = m_time;
    m_cur.subject = m_subject;
    m_cur.range = m_range;
    m_cur.bearing = m_bearing;
    multirobot_measurement[index].push_back(m_cur);
  }
  file1.close();
  file1_2.close();
}


void readOdometry(int index, string path_odometry){
  ifstream file2(path_odometry);
  ifstream file2_2(path_odometry);
  int line_count2 = 0;
  string line2;
  double o_time, o_forward_velocity, o_angular_velocity;
  odometry o_cur;
  while (getline(file2, line2))
    ++line_count2;
  for (int i = 0; i < line_count2; i++)
  {
    file2_2 >> o_time >> o_forward_velocity>>o_angular_velocity;
    o_cur.id = i;
    o_cur.time = o_time;
    o_cur.forward_velocity = o_forward_velocity;
    o_cur.angular_veolocity = o_angular_velocity;
    multirobot_odometry[index].push_back(o_cur);
  }
  file2.close();
  file2_2.close();

}



void readGroundtruth(int index, string path_groundtruth){
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

  
//n is the number of robots
slam_project::Robot_Odometry sendMultiMsg_Odometry(int j, int n)
{
  cout<<"j: "<<endl;
  slam_project::Robot_Odometry msg_odometry;

  msg_odometry.num.resize(n);
  msg_odometry.time.resize(n);
  msg_odometry.forward_velocity.resize(n);
  msg_odometry.angular_velocity.resize(n);
  msg_odometry.measure.resize(n);
  msg_odometry.robot_num = n;
  msg_odometry.id = j;
  for (int i=0; i<n; i++){
    msg_odometry.time[i] = multirobot_odometry[i][j].time;
    msg_odometry.forward_velocity[i] = multirobot_odometry[i][j].forward_velocity;
    msg_odometry.angular_velocity[i] = multirobot_odometry[i][j].angular_veolocity;
  }

  int count = 0;
  vector<vector<int> > msg_subject(n);
  vector<vector<double> > msg_range(n);
  vector<vector<double> > msg_bearing(n);

  //only robot1, robot2, ..., robotn's data is transmitted
  for (int i=0; i<n; i++){
    count = 0;
    cout<<"kk[i]: "<<kk[i]<<endl;
    if (multirobot_measurement[i][kk[i]].time == msg_odometry.time[i]){
      while (multirobot_measurement[i][kk[i]].time == msg_odometry.time[i]){
        bool flag = true;
        //n=2
        //m=3,4,5
        for (int m=n+1; m<=5; m++){
          if (multirobot_measurement[i][kk[i]].subject == subject[m]){
            flag = false;
            break;
          }
        }
        if (flag){
          msg_subject[i].push_back(multirobot_measurement[i][kk[i]].subject);
          msg_range[i].push_back(multirobot_measurement[i][kk[i]].range);
          msg_bearing[i].push_back(multirobot_measurement[i][kk[i]].bearing);
          count++;
        }
        kk[i]++;
      }   
    }

    if (count)
    {
      msg_odometry.measure[i].subject.resize(count);
      msg_odometry.measure[i].range.resize(count);
      msg_odometry.measure[i].bearing.resize(count);
      for (int l = 0; l < count; l++)
      {
        msg_odometry.measure[i].subject[l] = msg_subject[i][l];
        msg_odometry.measure[i].range[l] = msg_range[i][l];
        msg_odometry.measure[i].bearing[l] = msg_bearing[i][l];
      }

    }
    msg_odometry.num[i] = count;

  }

  cout << "j: " << j << " time: " << msg_odometry.time[0] << " " <<
          "num: " << msg_odometry.num[0] <<endl;

  cout << "j: " << j << " time: " << msg_odometry.time[1] << " " <<
          "num: " << msg_odometry.num[1] <<endl;    


  return msg_odometry;
}

//single robot

void readData()
{
  //read data
  //vector<measure> robot_measurment;
  string path = ros::package::getPath("slam_project");
  ifstream file1(path + "/data/newRobot1_Measurement.txt");
  ifstream file1_2(path + "/data/newRobot1_Measurement.txt");
  int line_count1 = 0;
  string line1;
  double m_time, m_range, m_bearing;
  int m_subject;
  measure m_cur;
  while (getline(file1, line1))
    ++line_count1;
  for (int i = 0; i < line_count1; i++)
  {
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
  ifstream file2(path + "/data/newRobot1_Odometry.txt");
  ifstream file2_2(path + "/data/newRobot1_Odometry.txt");
  int line_count2 = 0;
  string line2;
  double o_time, o_forward_velocity, o_angular_velocity;
  odometry o_cur;
  while (getline(file2, line2))
    ++line_count2;
  for (int i = 0; i < line_count2; i++)
  {
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

  //barcode
  //vector<int> subject;
  ifstream file4(path + "/data/Barcodes.dat");
  ifstream file4_2(path + "/data/Barcodes.dat");
  subject.push_back(0);
  int line_count4 = 0;
  string line4;
  int subject_index, barcode_index;
  while (getline(file4, line4))
    ++line_count4;
  for (int i = 0; i < line_count4; i++)
  {
    file4_2 >> subject_index>>barcode_index;
    subject.push_back(barcode_index);
  }
  file4.close();
  file4_2.close();

  //Landmark_Groundtruth
  //vector<landmark> landmark_groundtruth;
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

bool add(slam_project::requestBarcode::Request &req,
        slam_project::requestBarcode::Response &res)
{
  if (req.a != 1) return false;
  int len = subject.size() - 1;
  res.barcode.resize(len + 1);
  res.barcode[0] = len;
  for (int i = 1; i <= len; i++)
  {
    res.barcode[i] = subject[i];
    ROS_INFO("sending back response: %d", (int) res.barcode[i]);
  }
  ROS_INFO("request barcode data");
  return true;
}



//multi robot
int main(int argc, char **argv)
{
  readData();
  
  ros::init(argc, argv, "data_reader"); //node name "data_reader"
  ros::NodeHandle node("~");
  
  int num_robots; //number of robots
  node.param<int>("data/num_robots", num_robots, 1);
  readMultiData(num_robots);

  ros::Publisher dataPublisher2 = node.advertise<slam_project::Robot_Odometry>("/publishMsg2", 100000);
  ros::ServiceServer service = node.advertiseService("requestData", add);
  ROS_INFO("Ready to send barcode data.");
  
  ros::Rate rate(100);
  ROS_INFO("start spinning");
  int j = 0;
  kk.resize(num_robots, 0);

  while (dataPublisher2.getNumSubscribers() == 0)
  {
  }
  while (ros::ok())
  {
//    cout<<j<<endl;
    if (j < 7000)
    {
//      dataPublisher2.publish(sendMsg_Odometry(j));
      dataPublisher2.publish(sendMultiMsg_Odometry(j, num_robots));
      j++;
    }

    ros::spinOnce(); // check for incoming messages
    rate.sleep();
  }

  return EXIT_SUCCESS;
}


/*int main(int argc, char **argv)
{
  readData();
  ros::init(argc, argv, "data_reader"); //node name "data_reader"
  ros::NodeHandle node;

  ros::Publisher dataPublisher2 = node.advertise<slam_project::Robot_Odometry>("/publishMsg2", 100000);
  ros::Publisher dataPublisher3 = node.advertise<slam_project::Robot_GroundTruth>("/publishMsg3", 1000);
  ros::ServiceServer service = node.advertiseService("requestData", add);
  ROS_INFO("Ready to send barcode data.");
  //  ros::spin();


  ros::Rate rate(100);
  ROS_INFO("start spinning");
  int j = 0;

  cout << "robot odometry size: " << robot_odometry.size() << endl;
  while (dataPublisher2.getNumSubscribers() == 0) // TODO?
  {

  }
  
  while (ros::ok())
  {
    //dataPublisher3.publish(sendMsg_GroundTruth(i));
    if (j < 7000)
    {
      dataPublisher2.publish(sendMsg_Odometry(j));
    }
    j++;
    ros::spinOnce(); // check for incoming messages
    rate.sleep();
//    cout << robot_odometry.size() << endl;
  }

  return EXIT_SUCCESS;
}
*/