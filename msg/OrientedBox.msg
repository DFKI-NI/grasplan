# Oriented 3-D bounding box enclosing the back-projected mask points.
std_msgs/Header header
geometry_msgs/Pose pose       # box centre; orientation is a yaw about the frame z axis
geometry_msgs/Vector3 size    # full extents along the box's local x, y, z
uint32 num_points             # number of cloud points the box encloses
float32 box_padding           # metres added to every side of size (0: the measured extent, or unknown)
