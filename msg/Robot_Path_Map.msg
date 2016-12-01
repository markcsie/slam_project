# TODO: Pose2D and PoseWithCovariance or PoseWithCovarianceStamped
float64[] time
int16[] robot_id
float64[] x
float64[] y
float64[] orientation

int16[] landmark_id
float64[] landmark_x
float64[] landmark_y
std_msgs/Float64MultiArray[] landmark_cov
int16 num

#robot real groundtruth
float64[] rx
float64[] ry
float64[] rorientation
int32 id

# particle pose
float64[] px
float64[] py
float64[] porientation