std_msgs/Header     header
# The name of the detector that detected the person (i.e frontalface, profileface)
string              name
string              object_id
geometry_msgs/Point pos
float64             reliability
float64[9]          covariance
byte                initialization

