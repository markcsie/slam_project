#include <vector>
#include <iostream>
#include <fstream>
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

//Time [s]    x [m]    y [m]    orientation [rad] 
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
