# Stability data

# User ID
string uid

# User Tineti "normalized" score (0-1)
#  
#  The Tinetti test has a gait score and a balance score totalling 28 points.
#  The lower the score, the higher the risk of falling.
#  
#  Tinneti score 	Risk of fall
#   ≤ 18 	          High
#   19-23 	          Moderate
#   ≥ 24 	          Low 

float64 tin

# Stability value (0-1)
float64 sta

# User pose for given stability (m)  
geometry_msgs/PoseStamped pose

