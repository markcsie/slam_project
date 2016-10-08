#include <vector>
#include <iostream>
#include <fstream>
using namespace std;

//Time [s]    Subject #    range [m]    bearing [rad] 
typedef struct slam_measure{
  	int id;
  	double time;
  	int subject;
  	double range;
  	double bearing;
}measure;

//Time [s]    forward velocity [m/s]    angular velocity[rad/s] 
typedef struct slam_odometry{
	int id;
	double time;
	double forward_velocity;
	double angular_veolocity;
}odometry;

//Time [s]    x [m]    y [m]    orientation [rad] 
typedef struct slam_groundtruth{
	int id;
	double time;
	double x;
	double y;
	double orientation;
}groundtruth;

//Subject #    x [m]    y [m]    x std-dev [m]    y std-dev [m] 
typedef struct slam_landmark_groundtruth{
	int id;
	int subject;
	double x;
	double y;
	double xstd_dev;
	double ystd_dev;
}landmark;
