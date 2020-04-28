#!/usr/bin/env python
# -*- coding: utf-8 -*-

from dynamic_reconfigure.server import Server
import rospy
import numpy as np
import math
import sys
from geometry_msgs.msg import Quaternion, TransformStamped
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64
from vibration_navigator_driver.cfg import WalkingStatusServerConfig
from vibration_navigator_msgs.msg import WalkingStatus

class WalkingStatusServer:

    def __init__( self ):

        rospy.init_node( 'walking_status_server_node' )

        # Static Parameter
        hz_duration = rospy.get_param('~hz_duration', 5.0 )

        self.buffer_acc_x_ = []
        self.buffer_acc_y_ = []
        self.buffer_acc_z_ = []
        self.buffer_gyro_x_ = []
        self.buffer_gyro_y_ = []
        self.buffer_gyro_z_ = []

        # publish rate
        topic_hz = 0 # hz
        self.counter_ = 0
        rospy.wait_for_message('~imu', Imu)
        sub = rospy.Subscriber( '~imu', Imu, self.callbackHz )
        rospy.loginfo('Measuring publish rate of topic {}'.format(sub.resolved_name))
        rospy.sleep(hz_duration)
        if self.counter_ == 0:
            rospy.logerr('No message')
            sys.exit(1)
        topic_hz = self.counter_ / hz_duration
        sub.unregister()
        rospy.loginfo('Publish rate of {} is {} Hz'.format( '~imu', topic_hz) )
        self.publish_duration_ = 1.0 / topic_hz

        #
        self.ave_acc_x_ = np.average( np.array( self.buffer_acc_x_ ) )
        self.ave_acc_y_ = np.average( np.array( self.buffer_acc_y_ ) )
        self.ave_acc_z_ = np.average( np.array( self.buffer_acc_z_ ) )
        self.ave_gyro_x_ = np.average( np.array( self.buffer_gyro_x_ ) )
        self.ave_gyro_y_ = np.average( np.array( self.buffer_gyro_y_ ) )
        self.ave_gyro_z_ = np.average( np.array( self.buffer_gyro_z_ ) )

        #
        self.server_cfg_ = Server( WalkingStatusServerConfig, self.callbackConfig )

        # callback registration
        self.subscriber_ = rospy.Subscriber( '~imu', Imu, self.callback )
        self.publisher_status_ = rospy.Publisher( '~status', WalkingStatus, queue_size=10 )
        self.publisher_score_ = rospy.Publisher( '~score', Float64, queue_size=10 )

        # ROS message
        self.msg_status_ = WalkingStatus()
        self.msg_score_ = Float64()

        #
        self.prev_timestamp_ = rospy.Time.now()
        self.prev_acc_x_ = 0.0
        self.prev_acc_y_ = 0.0
        self.prev_acc_z_ = 0.0
        self.prev_gyro_x_ = 0.0
        self.prev_gyro_y_ = 0.0
        self.prev_gyro_z_ = 0.0
        self.prev_score_ = 0.0

        rospy.loginfo( 'Initialization finished.' )

    def callbackHz( self, msg ):

        self.counter_ += 1
        self.buffer_acc_x_.append( msg.linear_acceleration.x )
        self.buffer_acc_y_.append( msg.linear_acceleration.y )
        self.buffer_acc_z_.append( msg.linear_acceleration.z )
        self.buffer_gyro_x_.append( msg.angular_velocity.x )
        self.buffer_gyro_y_.append( msg.angular_velocity.y )
        self.buffer_gyro_z_.append( msg.angular_velocity.z )

    def callbackConfig( self, config, level ):

        rospy.loginfo('callback config called')

        self.parameter_coef_acc_x_ = config['coef_acc_x']
        self.parameter_coef_acc_y_ = config['coef_acc_y']
        self.parameter_coef_acc_z_ = config['coef_acc_z']
        self.parameter_coef_acc_d_x_ = config['coef_acc_derivative_x']
        self.parameter_coef_acc_d_y_ = config['coef_acc_derivative_y']
        self.parameter_coef_acc_d_z_ = config['coef_acc_derivative_z']
        self.parameter_coef_gyro_x_ = config['coef_gyro_x']
        self.parameter_coef_gyro_y_ = config['coef_gyro_y']
        self.parameter_coef_gyro_z_ = config['coef_gyro_z']
        self.parameter_coef_gyro_d_x_ = config['coef_gyro_derivative_x']
        self.parameter_coef_gyro_d_y_ = config['coef_gyro_derivative_y']
        self.parameter_coef_gyro_d_z_ = config['coef_gyro_derivative_z']
        self.parameter_lpf_factor_ = config['lpf_factor']
        self.parameter_thrsh_score_ = config['threshold']

        return config

    def callback( self, msg ):

        acc_x = msg.linear_acceleration.x - self.ave_acc_x_
        acc_y = msg.linear_acceleration.y - self.ave_acc_y_
        acc_z = msg.linear_acceleration.z - self.ave_acc_z_
        gyro_x = msg.angular_velocity.x - self.ave_gyro_x_
        gyro_y = msg.angular_velocity.y - self.ave_gyro_y_
        gyro_z = msg.angular_velocity.z - self.ave_gyro_z_

        derivative_acc_x = ( acc_x - self.prev_acc_x_ ) / self.publish_duration_
        derivative_acc_y = ( acc_y - self.prev_acc_y_ ) / self.publish_duration_
        derivative_acc_z = ( acc_z - self.prev_acc_z_ ) / self.publish_duration_
        derivative_gyro_x = ( gyro_x - self.prev_gyro_x_ ) / self.publish_duration_
        derivative_gyro_y = ( gyro_y - self.prev_gyro_y_ ) / self.publish_duration_
        derivative_gyro_z = ( gyro_z - self.prev_gyro_z_ ) / self.publish_duration_

        self.prev_acc_x_ = acc_x
        self.prev_acc_y_ = acc_y
        self.prev_acc_z_ = acc_z
        self.prev_gyro_x_ = gyro_x
        self.prev_gyro_y_ = gyro_y
        self.prev_gyro_z_ = gyro_z
        self.prev_timestamp_ = msg.header.stamp

        score_cur = self.parameter_coef_acc_x_ * abs( acc_x ) \
                  + self.parameter_coef_acc_y_ * abs( acc_y ) \
                  + self.parameter_coef_acc_z_ * abs( acc_z ) \
                  + self.parameter_coef_acc_d_x_ * abs( derivative_acc_x ) \
                  + self.parameter_coef_acc_d_y_ * abs( derivative_acc_y ) \
                  + self.parameter_coef_acc_d_z_ * abs( derivative_acc_z ) \
                  + self.parameter_coef_gyro_x_ * abs( gyro_x ) \
                  + self.parameter_coef_gyro_y_ * abs( gyro_y ) \
                  + self.parameter_coef_gyro_z_ * abs( gyro_z ) \
                  + self.parameter_coef_gyro_d_x_ * abs( derivative_gyro_x ) \
                  + self.parameter_coef_gyro_d_y_ * abs( derivative_gyro_y ) \
                  + self.parameter_coef_gyro_d_z_ * abs( derivative_gyro_z )

        score = self.parameter_lpf_factor_ * score_cur + ( 1 - self.parameter_lpf_factor_ ) * self.prev_score_
        self.prev_score_ = score

        self.msg_score_.data = score

        # check if calculated power is above threshold or not.
        if score > self.parameter_thrsh_score_:
            self.msg_status_.phase = WalkingStatus.SWING
        else:
            self.msg_status_.phase = WalkingStatus.STANCE

        self.publisher_status_.publish( self.msg_status_ )
        self.publisher_score_.publish( self.msg_score_ )

def main():
    obj = WalkingStatusServer()
    rospy.spin()

if __name__=='__main__':
    main()
