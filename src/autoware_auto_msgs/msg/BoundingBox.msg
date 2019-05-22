# Oriented bounding box representation
geometry_msgs/Point32 centroid
geometry_msgs/Point32 size
Quaternion32 orientation
geometry_msgs/Point32[4] corners
# can hold arbitrary value, e.g. likelihood, area, perimeter
float32 value
# can hold arbitrary value e.g. 0=car
uint32 label
