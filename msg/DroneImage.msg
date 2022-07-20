Header header

# Image section
# http://docs.ros.org/en/api/sensor_msgs/html/msg/Image.html

uint32  image_height          # image height, that is, number of rows
uint32  image_width           # image width, that is, number of columns
string  image_encoding        # Encoding of pixels -- channel meaning, ordering, size
                              # taken from the list of strings in include/sensor_msgs/image_encodings.h
uint8   image_is_bigendian    # is this data bigendian?
uint32  image_step            # Full row length in bytes
uint8[] image_data            # actual matrix data, size is (step * rows)

# GPS section (from /mavros/global_position/global)
# http://docs.ros.org/en/api/sensor_msgs/html/msg/NavSatFix.html

float64 drone_latitude        # Latitude [degrees]. Positive is north of equator; negative is south.
float64 drone_longitude       # Longitude [degrees]. Positive is east of prime meridian; negative is west.
float64 drone_altitude        # Altitude [m]. Positive is above the WGS 84 ellipsoid (quiet NaN if no altitude is available).


# Change package.xml in iq_sim and compile with cd ~/catkin_ws && catkin build
# http://wiki.ros.org/ROS/Tutorials/CreatingMsgAndSrv