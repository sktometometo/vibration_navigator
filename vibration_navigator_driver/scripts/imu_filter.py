#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import tf2_geometry_msgs
from geometry_msgs.msg import TransformStamped, Vector3Stamped
from sensor_msgs.msg import Imu


class ImuFilter:

    def __init__( self ):

        rospy.init_node( 'walking_status_server_node' )
        rospy.Subscriber( '~imu', Imu, self.callback )
        self.publisher_ = rospy.Publisher( '~imu_filtered', Imu, queue_size=10 )

    def callback( self, msg ):

        transform_inverted = TransformStamped()
        transform_inverted.child_frame_id = msg.header.frame_id
        transform_inverted.transform.rotation = msg.orientation
        transform_inverted.transform.rotation.w = - msg.orientation.w
        #
        ang_filtered = tf2_geometry_msgs.do_transform_vector3(
                                        Vector3Stamped( msg.header, msg.angular_velocity),
                                        transform_inverted )
        acc_filtered = tf2_geometry_msgs.do_transform_vector3(
                                        Vector3Stamped( msg.header, msg.linear_acceleration),
                                        transform_inverted )
        #
        msg_pub = Imu()
        msg_pub.header = msg.header
        msg_pub.orientation.w = 1.0
        msg_pub.angular_velocity = ang_filtered.vector
        msg_pub.linear_acceleration = acc_filtered.vector
        #
        self.publisher_.publish( msg_pub )

def main():
    obj = ImuFilter()
    rospy.spin()

if __name__=='__main__':
    main()
