#!/usr/bin/env python
# -*- coding: utf-8 -*-

from dynamic_reconfigure.server import Server
import rospy
import numpy as np
from geometry_msgs.msg import Quaternion, TransformStamped
from vibration_navigator_driver.cfg import WalkingStatusServerConfig
from vibration_navigator_msgs.msg import WalkingStatus, ImuSpectrum, Power


class WalkingStatusServer:

    def __init__( self ):

        rospy.init_node( 'walking_status_server_node' )

        # callback registration
        self.subscriber_ = rospy.Subscriber( '~spectrum', ImuSpectrum, self.callback )
        self.publisher_status_ = rospy.Publisher( '~status', WalkingStatus, queue_size=10 )
        self.publisher_power_ = rospy.Publisher( '~power', Power, queue_size=10 )
        self.server_cfg_ = Server( WalkingStatusServerConfig, self.callbackConfig )

        #
        self.msg_status_ = WalkingStatus()
        self.msg_power_ = Power()

        # Static Parameter
        self.threshold_freq_ = rospy.get_param('~threshold_freq', 1.0) # Hz
        self.threshold_power_ = rospy.get_param('~threshold_power', 1.0) #

        #
        rospy.loginfo( 'Initialization finished.' )

    def callbackConfig( self, config, level ):

        self.threshold_freq_ = config['threshold_frequency']
        self.threshold_power_ = config['threshold_power']

        return config

    def callbackHz( self, msg ):

        self.counter += 1

    def callback( self, msg ):

        #
        spctr = np.array( [ msg.acc_x, msg.acc_y, msg.acc_z ] )
        freq = np.array( msg.frequency )

        # calculate spectrum ( in power )
        power_x = np.sum(spctr[0][np.abs(freq) > self.threshold_freq_] ** 2 / np.abs( freq )[np.abs(freq) > self.threshold_freq_])
        power_y = np.sum(spctr[1][np.abs(freq) > self.threshold_freq_] ** 2 / np.abs( freq )[np.abs(freq) > self.threshold_freq_])
        power_z = np.sum(spctr[2][np.abs(freq) > self.threshold_freq_] ** 2 / np.abs( freq )[np.abs(freq) > self.threshold_freq_])

        # calculate power of specter of frequency above threshold_freq
        power = power_x + power_z
        self.msg_power_.header = msg.header
        self.msg_power_.data = power

        # check if calculated power is above threshold or not.
        if power > self.threshold_power_:
            self.msg_status_.phase = WalkingStatus.SWING
        else:
            self.msg_status_.phase = WalkingStatus.STANCE

        self.publisher_status_.publish( self.msg_status_ )
        self.publisher_power_.publish( self.msg_power_ )

def main():
    obj = WalkingStatusServer()
    rospy.spin()

if __name__=='__main__':
    main()
