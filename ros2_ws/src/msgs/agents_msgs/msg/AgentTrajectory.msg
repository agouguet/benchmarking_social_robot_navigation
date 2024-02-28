# Message defining an trajectory of a agent

std_msgs/Header      header         # Age of the track
uint64               id             # Unique ID for each agent

# Poses of the trajectory
geometry_msgs/Point[] poses