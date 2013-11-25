uint32 index
string name
float64 mass
int8 body_type

# ORS transformation
geometry_msgs/Transform transform
geometry_msgs/Twist twist

geometry_msgs/Vector3 com

# Wrench wrench
geometry_msgs/Vector3 force
geometry_msgs/Vector3 torque

# shapes
ors_msgs/Shape[] shapes
