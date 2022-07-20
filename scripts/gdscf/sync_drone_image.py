#!/usr/bin/env python
import message_filters
import rospy

from std_msgs.msg import Header
from std_msgs.msg import Float64
from sensor_msgs.msg import Image
from sensor_msgs.msg import NavSatFix

from iq_sim.msg import DroneImage

# Initialize the global GPS data dictionary
gps_data = {"latitude":"", "longitude":"", "altitude":""}

# This callback is used to get the GPS readings in advance, so we can link the image to the
# last-known GPS data available
def gps_rel_alt_callback(gps, rel_alt):
    global gps_data
    gps_data = {"latitude": gps.latitude, "longitude": gps.longitude, "altitude": rel_alt.data}
    # Debug output:
    #rospy.loginfo("#### GPS / Relative Altitude read value: " + str(gps_data))

# This callback is used to merge image and GPS data into a new type 'iq_sim.DroneImage'
def image_callback(image):
    #rospy.loginfo("### Image read.")
    # DroneImage type is:
    #image_height        
    #image_width         
    #image_encoding      
    #image_is_bigendian  
    #image_step          
    #image_data          
    #drone_latitude      
    #drone_longitude     
    #drone_altitude      
    
    global gps_data
    # We copy the GPS data to minimize the potential risk of accessing half-written data
    # TODO: Check how python handles this in more depth
    gps_prev_data = gps_data
    
    # Check if GPS data is not empty
    if (gps_prev_data["latitude"] == "") or (gps_prev_data["longitude"] == "") or (gps_prev_data["altitude"] == ""):
        rospy.loginfo("### Image received, but GPS is not ready (sync_drone_image).")
        return

    pub_drone_image.publish(image.header,
                            image.height,
                            image.width,
                            image.encoding,
                            image.is_bigendian,
                            image.step,
                            image.data,
                            gps_prev_data["latitude"],
                            gps_prev_data["longitude"],
                            gps_prev_data["altitude"])
    # Debug output:
    #rospy.loginfo("### DroneImage published to /sync_drone_image topic.")

if __name__ == "__main__":
    rospy.init_node('sync_drone_image')
    
    rospy.loginfo("### Subscribing to image_raw.")
    rospy.Subscriber('/webcam_down/image_raw', Image, image_callback)
    
    gps_global_sub = message_filters.Subscriber('/mavros/global_position/global', NavSatFix)
    rel_alt_sub = message_filters.Subscriber('/mavros/global_position/rel_alt', Float64)
    
    rospy.loginfo("### Initializing TimeSynchronizer(gps, rel_alt)")
    # slop = 0.1 -> 100 ms delay between messages is permissible, at most
    # allow_headerless -> because rel_alt doesn't have a header
    # https://docs.ros.org/en/api/message_filters/html/python/#message_filters.TimeSynchronizer
    gps_rel_alt = message_filters.ApproximateTimeSynchronizer([gps_global_sub, rel_alt_sub], queue_size=100, slop=0.1, allow_headerless=True)
    gps_rel_alt.registerCallback(gps_rel_alt_callback)

    rospy.loginfo("### Initializing Publisher\n")
    pub_drone_image = rospy.Publisher('/sync_drone_image', DroneImage, queue_size=100)
    
    # 10hz. Same as GPS which is the lowest
    rate = rospy.Rate(10) 
        
    rospy.spin()
