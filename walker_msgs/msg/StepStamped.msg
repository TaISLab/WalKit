# (m)  
geometry_msgs/PointStamped position

# (0-100) Detection confidence. 0 by default
float32 confidence 0

# (bool) If this is false, Ignore speed and load fields
bool tracked false

# (m/s) leg motion speed (same frame than position)
geometry_msgs/Point speed 

# (kg) weight on foot. -1 means it hasn't been calculated or no single load on it.
float32 load -1
