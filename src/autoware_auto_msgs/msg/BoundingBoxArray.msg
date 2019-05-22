# Message for a full set of bounding boxes
std_msgs/Header header
BoundingBox[256] boxes
uint32 size 

# This capacity is chosen empirically as a safety factor on top of
# "average" number of objects, 50-70
uint32 CAPACITY=256
