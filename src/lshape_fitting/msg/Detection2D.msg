std_msgs/Header header

# 차량 ID
int32 vehicle_id

# x, y, yaw (z=0)
geometry_msgs/Pose pose

# L-shape 등에서 나온 크기
float32 length
float32 width

# 검출 신뢰도(0~1), 없으면 1.0
float32 confidence