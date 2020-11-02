# This represents information about a single victim

int32 id                          # Unique ID for this victim
geometry_msgs/PointStamped point  # Victim location
sensor_msgs/Image image           # Image of the victim
string description                # Description of this victim (optional)
