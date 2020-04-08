#!/usr/bin/env python
# -*- coding: utf-8 -*-

from dynamic_reconfigure.server import Server
import rospy
import numpy as np
from sensor_msgs.msg import Imu
import sys
from vibration_navigator_msgs.msg import ImuSpectrum


class ImuSpectrumPublisher:

    def __init__( self ):

        rospy.init_node( 'imu_spectrum_publisher_node' )

        # Static Parameter
        hz_duration = rospy.get_param('~hz_duration', 5.0) # sec
        buffer_duration = rospy.get_param('~buffer_duration', 1.0) # sec

        # ROS message
        self.msg_spectrum_ = ImuSpectrum()

        # measure topic rate
        topic_hz = 0 # hz
        self.counter_ = 0
        sub = rospy.Subscriber( '~imu', Imu, self.callbackHz )
        rospy.sleep(hz_duration)
        if self.counter_ == 0:
            ros.error('No message')
            sys.exit(1)
        topic_hz = self.counter_ / hz_duration
        sub.unregister()
        rospy.loginfo('Publish rate of {} is {} Hz'.format( '~imu', topic_hz) )

        # callback registration
        self.subscriber_ = rospy.Subscriber( '~imu', Imu, self.callback )
        self.publisher_spectrum_ = rospy.Publisher( '~spectrum', ImuSpectrum, queue_size=10 )

        self.array_acc_buffer = np.zeros(( 3, int(buffer_duration*topic_hz) ))
        self.array_acc_spctr = np.zeros(( 3, int(buffer_duration*topic_hz) ))
        self.array_frequency = np.fft.fftfreq( n=int(buffer_duration*topic_hz), d=(1.0/topic_hz) )

        self.msg_spectrum_.frequency = self.array_frequency.tolist()

        rospy.loginfo('initialization finished.')

    def callbackHz( self, msg ):

        self.counter_ += 1

    def callback( self, msg ):

        # update buffer
        self.array_acc_buffer[:,:-1] = self.array_acc_buffer[:,1:]
        self.array_acc_buffer[0,-1] = msg.linear_acceleration.x
        self.array_acc_buffer[1,-1] = msg.linear_acceleration.y
        self.array_acc_buffer[2,-1] = msg.linear_acceleration.z

        # calculate spectrum ( in power )
        self.array_acc_spctr = np.abs(np.fft.fft( self.array_acc_buffer, axis=1 ))[:]

        #
        self.msg_spectrum_.header = msg.header
        self.msg_spectrum_.acc_x = self.array_acc_spctr[0].tolist()
        self.msg_spectrum_.acc_y = self.array_acc_spctr[1].tolist()
        self.msg_spectrum_.acc_z = self.array_acc_spctr[2].tolist()

        self.publisher_spectrum_.publish( self.msg_spectrum_ )


def main():

    obj = ImuSpectrumPublisher()
    rospy.spin()

if __name__ == '__main__':
    main()
