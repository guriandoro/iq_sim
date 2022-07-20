#!/usr/bin/python

import rospy
from std_msgs.msg import String
from std_msgs.msg import Float64
from sensor_msgs.msg import Image
from iq_sim.msg import DroneImage

# To parse arguments
import sys
# To deal with hex in data.data array
import struct
# PostgreSQL driver
import psycopg2

# Connection to postgres instance (global)
conn = None
# Flag to know if drone has taken off (global)
in_flight = 0

def print_debug(message=""):
    if debug:
        print(message)

def connect():
    try:
        print_debug('### Connecting to the PostgreSQL database')
        global conn        
        
        # Get connection (tune connection string as needed).
        # TODO: add success/error checks
        conn = psycopg2.connect(host="192.168.1.3", dbname="gdscf", user="postgres", password="postgres")
    
        if debug:
            # Create a cursor.
            cur = conn.cursor()
            # Execute a statement.
            cur.execute('SELECT version()')
            # Display the PostgreSQL database server version.
            db_version = cur.fetchone()
            print_debug('### PostgreSQL database version: '+str(db_version))

    except (Exception, psycopg2.DatabaseError) as error:
        print_debug(error)
        raise Exception('### Connection to database not possible.')

def callback(data):
    global conn
    global in_flight
    
    if in_flight == 0:
        # The drone is taking off, or has taken off.
        # We hard-code mission min altitude as 8 meters (change if needed).
        if int(data.drone_altitude) > 8:
            in_flight = 1
            print_debug("### Enabling flight. in_flight: "+str(in_flight)+" rel_alt: "+str(data.drone_altitude))
    else:
        # The drone is landing.
        # We hard-code mission min altitude as 8 meters (change if needed).
        if int(data.drone_altitude) < 8:
            in_flight = 0
            print_debug("### Disabling flight. in_flight: "+str(in_flight)+" rel_alt: "+str(data.drone_altitude))

    # If the drone is not flying, don't record image.
    if (in_flight == 0) && debug:
        rospy.loginfo("### Drone is not flying. Not recording received image.")
        return

    ### How to parse header data:
    # http://docs.ros.org/en/api/std_msgs/html/msg/Header.html
    if debug:
        rospy.loginfo("The sample time is: %s", str(data.header.stamp.secs)+"."+str(data.header.stamp.nsecs))    
        rospy.loginfo("The sequence number is: " + str(data.header.seq))
    
    ### How to parse image data:
    # http://docs.ros.org/en/api/sensor_msgs/html/msg/Image.html
    cell_R = data.image_data[0]
    cell_G = data.image_data[1]
    cell_B = data.image_data[2]
    if debug:
        rospy.loginfo("The RGB values for first pixel are: %s, %s, %s", cell_R, cell_G, cell_B)
        rospy.loginfo("drone id: %s and mission_id: %s", drone_id, mission_id)
    
    if conn is None:
        rospy.logerror("### Connection to the database was lost.")
        raise Exception('No database connection.')

    cur = conn.cursor()
    
    # Insert into control table.
    insert_query = """ INSERT INTO drone_camera (drone_id, mission_id, secs, nsecs, cell_R, cell_G, cell_B) VALUES (%s,%s,%s,%s,%s,%s,%s)"""
    row = (drone_id, mission_id, data.header.stamp.secs, data.header.stamp.nsecs, cell_R, cell_G, cell_B)
    cur.execute(insert_query, row)
    conn.commit()
    
    if debug:
        count = cur.rowcount
        rospy.loginfo("Inserted in drone_camera table: "+str(count)+" records")
    
    # Insert into table with all metadata and data.
    insert_query = """ INSERT INTO drone_camera_blob (drone_id, mission_id, secs, nsecs, latitude, longitude, altitude, image) VALUES (%s,%s,%s,%s,%s,%s,%s,%s)"""
    row = (drone_id, mission_id, data.header.stamp.secs, data.header.stamp.nsecs, data.drone_latitude, data.drone_longitude, data.drone_altitude, data.image_data)
    cur.execute(insert_query, row)
    conn.commit()
    
    if debug:
        count = cur.rowcount
        rospy.loginfo("Inserted in drone_camera_blob table: "+str(count)+" records")
        rospy.loginfo("")

def listener():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('drone_camera_listener', anonymous=True)
    rospy.Subscriber('/sync_drone_image', DroneImage, callback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    # Default drone info (will be overwritten).
    drone_id = 1
    mission_id = 1
    # Default debug outputs. Change to False if less verbosity is needed.
    debug = True

    print_debug("### Arguments passed: "+str(len(sys.argv)))

    if len(sys.argv) > 2:
        # We assume first argument is drone_id
        drone_id = int(sys.argv[1])
        print_debug("### Drone ID: "+str(drone_id))
        # We assume second argument is mission_id
        mission_id = int(sys.argv[2])
        print_debug("### Mission ID: "+str(mission_id))
    else:
        raise Exception('ERROR: drone and mission IDs should be passed as first and second arguments.')

    connect()
    print_debug("### Connected to PostgreSQL")
    
    listener()

    print_debug("### Ending drone_camera_listener")

    if conn is not None:
        conn.close()
        print_debug('### Database connection closed.')
    else:
        print_debug('### Database connection was null.')
