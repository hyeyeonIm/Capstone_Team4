#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan  

def lidar_callback(data):
    data.header.stamp = rospy.Time.now()

    pub.publish(data)

if __name__ == '__main__':
    rospy.init_node('lidar_timestamp_modifier')
    
  
    pub = rospy.Publisher('/scan', LaserScan, queue_size=10)
    
 
    rospy.Subscriber('/scan', LaserScan, lidar_callback)
    

    rospy.spin()
