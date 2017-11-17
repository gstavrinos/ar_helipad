#!/usr/bin/env python
import tf
import math
import rospy
import untangle
from tf2_msgs.msg import TFMessage
from visualization_msgs.msg import Marker
from ar_track_alvar_msgs.msg import AlvarMarkers

mode = 0
width = 0.0
height = 0.0
top_left_x = 0
top_left_y = 0
marker_size = 0.0
master_marker = '0'
tf_broadcaster = None
marker_publisher = None

def init():
    global marker_publisher, master_marker, tf_broadcaster
    global width, height, top_left_x, top_left_y, mode, marker_size
    rospy.init_node('ar_helipad')
    # Mode 0 : Helipad with 2 ar tags in the corners
    # Mode 1 : Helipad that consists only of a ar marker
    mode = rospy.get_param("~mode", 0)
    if mode == 0:
        xml = rospy.get_param("~xml_file", "bundles/helipad1.xml")
        xml_file = untangle.parse(xml)
        master_marker = xml_file.multimarker.marker[0]['index']
        top_left_x = float(xml_file.multimarker.marker[0].corner[3]['x'])
        top_left_y = float(xml_file.multimarker.marker[0].corner[3]['y'])
        bottom_right_x = float(xml_file.multimarker.marker[1].corner[1]['x'])
        bottom_right_y = float(xml_file.multimarker.marker[1].corner[1]['y'])
        width = abs(bottom_right_x) + abs(top_left_x)
        height = abs(bottom_right_y) + abs(top_left_y)
        print 'The helipad has a width of ' + str(width) + 'cm.'
        print 'The helipad has a height of ' + str(height) + 'cm.'
        width /= 100
        height /= 100
        top_left_x /= 100
        top_left_y /= 100
    elif mode == 1:
        master_marker = rospy.get_param("~marker_index", '3')
        marker_size = rospy.get_param("~marker_size", 5.4)
    tf_broadcaster = tf.TransformBroadcaster()
    marker_publisher = rospy.Publisher('ar_helipad_marker', Marker, queue_size=1)
    rospy.Subscriber("tf", TFMessage, tf_callback)
    while not rospy.is_shutdown():
        rospy.spin()

def tf_callback(tf2):
    global master_marker, tf_broadcaster
    global width, height, mode, marker_size
    for tf in tf2.transforms:
        if tf.child_frame_id == 'ar_marker_' + master_marker:
            if mode == 0:
                tf_broadcaster.sendTransform(
                    (width / 2 + top_left_x, -height / 2 + top_left_y, 0),
                    (0, 0, 0, 1.0),
                    rospy.Time.now(),
                    'helipad',
                    'ar_marker_' + master_marker)

                marker = Marker()
                marker.header.frame_id = 'ar_marker_' + master_marker
                marker.header.stamp = rospy.Time.now()
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
                marker.pose.position.x = width / 2 + top_left_x
                marker.pose.position.y = -height / 2 + top_left_y
                marker.pose.position.z = 0
                marker.pose.orientation.x = 0
                marker.pose.orientation.y = 0
                marker.pose.orientation.z = 0
                marker.pose.orientation.w = 1.0
                # TODO change duration here?
                marker.lifetime = rospy.Duration(1);
                marker_publisher.publish(marker)
            elif mode == 1:
                tf_broadcaster.sendTransform(
                    (0, 0, 0),
                    (0, 0, 0, 1.0),
                    rospy.Time.now(),
                    'helipad',
                    'ar_marker_' + master_marker)

                marker = Marker()
                marker.header.frame_id = 'ar_marker_' + master_marker
                marker.header.stamp = rospy.Time.now()
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
                marker.scale.x = marker_size / 100
                marker.scale.y = marker_size / 100
                marker.scale.z = 0.001
                marker.pose.position.x = 0
                marker.pose.position.y = 0
                marker.pose.position.z = 0
                marker.pose.orientation.x = 0
                marker.pose.orientation.y = 0
                marker.pose.orientation.z = 0
                marker.pose.orientation.w = 1.0
                # TODO change duration here?
                marker.lifetime = rospy.Duration(1);
                marker_publisher.publish(marker)

        else:
            # TODO publish last seen tf, just to keep its tf alive?
            pass

if __name__ == '__main__':
    init() 