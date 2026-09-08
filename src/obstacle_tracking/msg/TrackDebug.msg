std_msgs/Header header
int32 track_id
int32 measurement_index
bool matched
bool confirmed
string status

geometry_msgs/Pose predicted_pose
float32 predicted_speed
float32 predicted_yaw_rate
float32 predicted_accel

geometry_msgs/Pose measurement_pose
float32 measurement_length
float32 measurement_width
float32 measurement_confidence

float32 residual_x
float32 residual_y
float32 residual_yaw
float32 innovation_distance
float32 mahalanobis_sq
float32 yaw_error
float32 size_cost
float32 total_cost

int32 hits
int32 missed
