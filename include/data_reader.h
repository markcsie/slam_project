#include <vector>
#include <iostream>
#include <fstream>
#include <sstream>
#include "slam_project/Robot_Path_Map.h"
#include "slam_project/Robot_Odometry.h"
#include "slam_project/Robot_Measure.h"
#include "slam_project/requestBarcode.h"
using namespace std;

//Time [s]    Subject #    range [m]    bearing [rad] 
struct measure{
  	int id;
  	double time;
  	int subject;
  	double range;
  	double bearing;
};

//Time [s]    forward velocity [m/s]    angular velocity[rad/s] 
struct odometry{
	int id;
	double time;
	double forward_velocity;
	double angular_veolocity;
};

//robot_groundtruth Time [s]    x [m]    y [m]    orientation [rad] 
struct groundtruth{
	int id;
	double time;
	double x;
	double y;
	double orientation;
};

//Subject #    x [m]    y [m]    x std-dev [m]    y std-dev [m] 
struct landmark{
	int id;
	int subject;
	double x;
	double y;
	double xstd_dev;
	double ystd_dev;
};

void readData();
bool add(slam_project::requestBarcode::Request &req,
         slam_project::requestBarcode::Response &res);


//multi robot
void readMultiData(int n);
slam_project::Robot_Odometry sendMultiMsg_Odometry(int j, int n);
void readMeasurement(int index, string path_measure);
void readOdometry(int index, string path_odometry);
void readGroundtruth(int index, string path_groundtruth);