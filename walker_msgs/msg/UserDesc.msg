# User description, entered once from the web GUI (walker_web_gui) and
# published latched (transient_local). Replaces the old ':'-separated
# std_msgs/String format ("age:height:weight:user_id:gender:tinetti:notes"),
# which every subscriber had to re-parse (and re-validate) by hand.

# as entered in the "User ID" field
string user_id

# "Masculino" or "Femenino", as entered in the GUI
string gender

# years
int32 age

# cm
int32 height

# Kg
int32 weight

# Tinetti POMA score (0-28). The lower the score, the higher the risk of
# falling; >=24 is considered low risk.
int32 tinetti_score

# free-text notes entered in the GUI
string description
