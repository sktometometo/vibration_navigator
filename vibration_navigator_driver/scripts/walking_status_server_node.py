#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
from geometry_msgs.msg import Quaternion, TransformStamped
from sensor_msgs.msg import Imu
from vibration_navigator_msgs.msg import WalkingStatus

def callback( msg ):

    quat = msg.orientation
    ang = msg.angular_velocity
    acc = msg.linear_acceleration

class WalkingStatusServer:

    def __init__( self ):

        rospy.init_node( 'walking_status_server_node' )
        self.subscriber_ = rospy.Subscriber( '~imu', Imu, self.callback )
        self.publisher_ = rospy.Publisher( '~status', WalkingStatus, queue_size=10 )

        self.threshold_freq = 1.0
        self.threshold_power = 1.0

        buffer_duration = rospy.get_param() # sec

        topic_hz = 0 # hz

        # measure with some method

        self.acc_buffer = np.zeros((3,(int)buffer_duration*topic_hz))
        self.acc_spctr = np.zeros((3,(int)buffer_duration*topic_hz))

        self.freq = (np.fft.fftfreq() / ( ( 1.0 / topic_hz ) * (int)buffer_duration*topic_hz ))[1:]

        self.msg_pub = WalkingStatus()

    def callback( self, msg ):

        # update buffer
        self.xarray[:-1] = self.xarray[1:]
        self.acc_buffer[:,:-1] = self.acc_buffer[:,1:]
        self.acc_buffer[0,-1] = msg.linear_acceleration.x
        self.acc_buffer[1,-1] = msg.linear_acceleration.y
        self.acc_buffer[2,-1] = msg.linear_acceleration.z

        # calculate spectrum ( in power )
        power_spctr = np.abs(np.fft.fft( self.acc_buffer, axis=1 )[1:])**2 / np.abs(self.freq)

        # calculate power of specter of frequency above threshold_freq
        power = power_spctr[ np.abs(self.freq) > self.threshold_freq ]

        # check if calculated power is above threshold or not.
        if power > threshold_power:
            self.msg_pub.phase = WalkingStatus.SWING
        else:
            self.msg_pub.phase = WalkingStatus.STANCE

        self.publisher_.publish( self.msg_pub )

def main():
    obj = WalkingStatusServer()
    rospy.spin()

if __name__=='__main__':
    main()
