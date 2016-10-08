#include <ros/ros.h>
#include "data_reader.h"
int main(int argc, char **argv) {
  	ros::init(argc, argv, "data_reader");
  	ros::NodeHandle node;
  
//read data
  	vector<measure> robot_measurment;
	ifstream file1("newRobot1_Measurement.txt");
	ifstream file1_2("newRobot1_Measurement.txt");
	int line_count1=0;
	string line1;
	double m_time, m_range, m_bearing;
	int m_subject;
	measure m_cur;
	while(getline(file1, line1))
		++line_count1;
	for (int i=0; i<line_count1; i++){
		file1_2>>m_time>>m_subject>>m_range>>m_bearing;
		m_cur.id=i;
		m_cur.time = m_time;
		m_cur.subject = m_subject;
		m_cur.range = m_range;
		m_cur.bearing = m_bearing;
		robot_measurment.push_back(m_cur);
	}
	file1.close();
	file1_2.close();

	//odometry
	vector<odometry> robot_odometry;
	ifstream file2("newRobot1_Odometry.txt");
	ifstream file2_2("newRobot1_Odometry.txt");
	int line_count2=0;
	string line2;
	double o_time, o_forward_velocity, o_angular_velocity;
	odometry o_cur;
	while(getline(file2, line2))
		++line_count2;
	for (int i=0; i<line_count2; i++){
		file2_2>>o_time>>o_forward_velocity>>o_angular_velocity;
		o_cur.id=i;
		o_cur.time = o_time;
		o_cur.forward_velocity = o_forward_velocity;
		o_cur.angular_veolocity = o_angular_velocity;
		robot_odometry.push_back(o_cur);
	}
	file2.close();
	file2_2.close();

	//groundtruth
	vector<groundtruth> robot_groundtruth;
	ifstream file3("newRobot1_Groundtruth.txt");
	ifstream file3_2("newRobot1_Groundtruth.txt");
	int line_count3=0;
	string line3;
	double g_time,g_x,g_y,g_orientation;
	groundtruth g_cur;
	while(getline(file3, line3))
		++line_count3;
	for (int i=0; i<line_count3; i++){
		file3_2>>g_time>>g_x>>g_y>>g_orientation;
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
	vector<int> subject;
	ifstream file4("Barcodes.dat");
	ifstream file4_2("Barcodes.dat");
	subject.push_back(0);
	int line_count4=0;
	string line4;
	int subject_index, barcode_index;
	while(getline(file4, line4))
		++line_count4;
	for (int i=0; i<line_count4; i++){
		file4_2>>subject_index>>barcode_index;
		subject.push_back(barcode_index);
	}
	file4.close();
	file4_2.close();

	//Landmark_Groundtruth
	vector<landmark> landmark_groundtruth;
	ifstream file5("Landmark_Groundtruth.dat");
	ifstream file5_2("Landmark_Groundtruth.dat");
	string line5;
	int line5_count;
	double l_x,l_y,l_xstd_dev,l_ystd_dev;
	int l_subject;
	landmark l_cur;
	while(getline(file5, line5))
		++line5_count;
	for (int i=0; i<line5_count; i++){
		file5_2>>l_subject>>l_x>>l_y>>l_xstd_dev>>l_ystd_dev;
		l_cur.subject = l_subject;
		l_cur.x=l_x;
		l_cur.y=l_y;
		l_cur.xstd_dev=l_xstd_dev;
		l_cur.ystd_dev=l_ystd_dev;
		landmark_groundtruth.push_back(l_cur);
	}
	file5.close();
	file5_2.close();  


	ros::Rate rate(1.0);
	ROS_INFO("start spinning");

	while (ros::ok()) {
		ros::spinOnce(); // check for incoming messages
	    rate.sleep();
	}
	  
    return EXIT_SUCCESS;
}
