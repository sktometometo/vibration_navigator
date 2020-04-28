// Standaerd C++ Library
#include <iostream>
// ROS
#include <ros/ros.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
// ROS message
#include <geometry_msgs/TransformStamped.h>
// Boost
#include <boost/bind.hpp>
// USER
#include "vibration_navigator_driver/contact_tf_publisher.h"

namespace vibration_navigator_driver {

    ContactTFPublisher::ContactTFPublisher(  ros::NodeHandle &nh,
                         ros::NodeHandle &nh_private,
                         tf2_ros::Buffer &tf_buffer,
                         tf2_ros::TransformBroadcaster &tf_broadcaster )
        : nh_(nh), nh_private_(nh_private), tf_buffer_(tf_buffer), tf_broadcaster_(tf_broadcaster)
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

        double timeout = 0.05;
        if ( this->nh_private_.hasParam("timeout_duration") ) {
            this->nh_private_.getParam("timeout_duration", timeout);
        }
        this->duration_timeout_ = ros::Duration(timeout);

        this->sub_walkingstatus_ = this->nh_.subscribe("walking_status", 1000, &ContactTFPublisher::callbackSub, this );

        /**
         *
         */
        this->walking_status_ = vibration_navigator_msgs::WalkingStatus::STANCE;

        ROS_INFO("Initialization finished.");
    }

    void ContactTFPublisher::spin()
    {
        ros::spin();
    }

    void ContactTFPublisher::callbackSub( const vibration_navigator_msgs::WalkingStatus& msg )
    {
        if ( this->walking_status_ != vibration_navigator_msgs::WalkingStatus::STANCE and
                msg.phase == vibration_navigator_msgs::WalkingStatus::STANCE ) {
            try {
                geometry_msgs::TransformStamped transform =
                    this->tf_buffer_.lookupTransform(
                            this->fixed_frame_id_,
                            this->reference_frame_id_,
                            ros::Time(0),
                            this->duration_timeout_
                            );
                transform.child_frame_id = this->output_frame_id_;
                transform.header.stamp = ros::Time::now();
                this->tf_broadcaster_.sendTransform(transform);
            } catch (tf2::TransformException &ex) {
                ROS_DEBUG("%s",ex.what());
            }
        }

        this->walking_status_ = msg.phase;

        return;
    }

}
