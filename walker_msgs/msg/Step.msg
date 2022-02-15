# (m)  As it is not stamped you need to check detectors' frame 
geometry_msgs/Point position

# (0-100) Detection confidence. 0 by default
float32 confidence 0

# (bool) If this is false, Ignore speed and load fields
bool tracked false

# (m/s) leg motion speed (also on detector's frame)
geometry_msgs/Point speed 

# (kg) weight on foot. -1 means it hasn't been calculated or no single load on it.
float32 load -1
