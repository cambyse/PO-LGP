string name
float64 mass

# ORS transformation
# TODO should we use ros Transformation? Transformation trans
geometry_msgs/Vector3 pos
geometry_msgs/Quaternion rot
geometry_msgs/Vector3 vel
geometry_msgs/Vector3 angvel

geometry_msgs/Vector3 com

#Wrench wrench
geometry_msgs/Vector3 force
geometry_msgs/Vector3 torque
