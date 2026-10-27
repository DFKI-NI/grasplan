std_msgs/Header header
string class_id
uint64 instance_id
geometry_msgs/Pose pose
geometry_msgs/Vector3 size
geometry_msgs/Vector3 min
geometry_msgs/Vector3 max
# metres the detector added to every side of size (min and max likewise); 0: size is the measured extent, or unknown
float32 box_padding
