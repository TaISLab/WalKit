# (m)  As it is not stamped you need to check detectors' frame 
geometry_msgs/PointStamped position

# (0-100) Detection confidence
float32 confidence 0

# (bool) Ignore the following if this is false
bool tracked false

# (m/s) leg motion speed
geometry_msgs/Point speed 

# (kg) weight on foot. If has default value, it hasn't been calculated
float32 load -1
