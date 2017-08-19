#!/usr/bin/env python
import math
import rospy
import untangle
from visualization_msgs.msg import Marker
from ar_track_alvar_msgs.msg import AlvarMarkers

marker_publisher = None
width = 0.0
height = 0.0
diagonal = 0.0

def init():
    global marker_publisher
    global width, height, diagonal
    rospy.init_node('ar_helipad')
    xml = rospy.get_param("~xml_file", "bundles/helipad1.xml")
    xml_file = untangle.parse(xml)
    top_left_x = float(xml_file.multimarker.marker[0].corner[3]['x'])
    top_left_y = float(xml_file.multimarker.marker[0].corner[3]['y'])
    bottom_right_x = float(xml_file.multimarker.marker[1].corner[1]['x'])
    bottom_right_y = float(xml_file.multimarker.marker[1].corner[1]['y'])
    width = bottom_right_x + abs(top_left_x)
    height = abs(bottom_right_y + top_left_y)
    diagonal = math.sqrt(width**2 + height**2)
    print 'The helipad has a width of ' + str(width) + 'cm.'
    print 'The helipad has a height of ' + str(height) + 'cm.'
    print 'The helipad has a diagonal of ' + str(diagonal) + 'cm.'
    width /= 100
    height /= 100
    diagonal /= 100
    marker_publisher = rospy.Publisher('ar_helipad_pose', Marker, queue_size=1)
    rospy.Subscriber("ar_pose_marker", AlvarMarkers, ar_callback)
    while not rospy.is_shutdown():
        rospy.spin()

def ar_callback(msg):
    global marker_publisher
    global width, height, diagonal
    marker = Marker()
    if len(msg.markers) > 0:
        marker.header.frame_id = msg.markers[0].header.frame_id
        marker.header.stamp = rospy.Time().now()
        marker.ns = 'ar_helipad'
        marker.id = 0
        # Based on http://docs.ros.org/api/visualization_msgs/html/msg/Marker.html
        # The shape code for the cube (rectangle) is 1
        # The add/modify action is 0
        marker.type = 1
        marker.action = 0
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        marker.scale.x = width
        marker.scale.y = height
        marker.scale.z = 0.001
        marker.pose = msg.markers[0].pose.pose
        #marker.pose.position.x = msg.markers[0].pose.pose.position.x + width / 2
        #marker.pose.position.y = msg.markers[0].pose.pose.position.y - diagonal / 2
        marker.lifetime = rospy.Duration(1);
        marker_publisher.publish(marker)
    #print msg

if __name__ == '__main__':
    init() 