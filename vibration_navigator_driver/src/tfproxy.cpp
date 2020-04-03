// Standaerd C++ Library
#include <iostream>
// ROS
#include <ros/ros.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
// ROS message
#include <geometry_msgs/TransformStamped.h>
// Boost
#include <boost/bind.hpp>
// USER
#include "vibration_navigator_driver/tfproxy.h"

namespace vibration_navigator_driver {

    bool TFProxy::init( 
                       ros::NodeHandle &nh,
                       ros::NodeHandle &nh_private,
                       tf2_ros::Buffer &tf_buffer,
                       tf2_ros::TransformBroadcaster &tf_broadcaster )
    {
        this->reference_frame_id_ = "reference_frame";
        if ( nh_private.hasParam("reference_frame_id") ) {
            nh_private.getParam("reference_frame_id", this->reference_frame_id_);
        }

        this->output_frame_id_ = "output_frame";
        if ( nh_private.hasParam("output_frame_id") ) {
            nh_private.getParam("output_frame_id", this->output_frame_id_);
        }

        this->fixed_frame_id_ = "fixed_frame";
        if ( nh_private.hasParam("fixed_frame_id") ) {
            nh_private.getParam("fixed_frame_id", this->fixed_frame_id_);
        }

        /**
         * Publisher and Subscriber generation
         */
        this->publisher_walking_status_ = nh_private.advertise<vibration_navigator_msgs::WalkingStatus>("output",10);

        /**
         *
         */
        this->initialized_ = false;

        return true;
    }

    void TFProxy::spin(
                           ros::NodeHandle &nh,
                           ros::NodeHandle &nh_private,
                           tf2_ros::Buffer &tf_buffer,
                           tf2_ros::TransformBroadcaster &tf_broadcaster )
    {
        // start broadcaster timer
        ros::Timer timer_tf = nh.createTimer( 
                                ros::Duration(0.1),
                                std::bind(
                                    TFProxy::callbackTimerTF,
                                    this,
                                    tf_buffer,
                                    tf_broadcaster,
                                    _1 )
                            );

        //
        ros::spin();
    }

    void TFProxy::callbackTimerTF(
                                        tf2_ros::Buffer &tf_buffer,
                                        tf2_ros::TransformBroadcaster &tf_broadcaster,
                                        const ros::TimerEvent& )
    {
        try {
            geometry_msgs::TransformStamped temp_transform =
                tf_buffer.lookupTransform(
                        this->reference_frame_id_,
                        this->fixed_frame_id_,
                        ros::Time(0)
                        );
            this->msg_transform_ = temp_transform;
            this->msg_transform_.header.stamp = ros::Time::now();
            if ( not this->initialized_ ) {
                this->initialized_ = true;
            }
        } catch (tf2::TransformException &ex) {
            this->msg_transform_.header.stamp = ros::Time::now();
        }
        if ( this->initialized_ ) {
            tf_broadcaster.sendTransform(transform);
        }
    }

}
