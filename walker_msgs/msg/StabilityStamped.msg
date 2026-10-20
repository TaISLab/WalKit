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

# Stability value (0-1): the margin below over the one that counts as fully stable
float64 sta

# User pose for given stability (m): the position of the centre of mass (an estimate of its projection on the floor)
geometry_msgs/PoseStamped pose

# Margin (m) of that position in the support polygon, the convex hull of the soles of the feet that carry load and of
# the handles: distance to the nearest edge, positive inside, negative outside, 0 if there is no polygon
float64 margin

# Contacts in the polygon: feet carrying load plus handles (0-4)
uint8 contacts
