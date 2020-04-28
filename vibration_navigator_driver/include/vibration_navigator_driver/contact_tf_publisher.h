#ifndef VIBRATION_NAVIGATOR_DRIVER_CONTACT_TF_PUBLISHER_H__
#define VIBRATION_NAVIGATOR_DRIVER_CONTACT_TF_PUBLISHER_H__

// Standaerd C++ Library
#include <iostream>
#include <vector>
// ROS
#include <ros/ros.h>
#include <ros/spinner.h>
#include <tf2_ros/transform_listener.h>
// ROS message
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/UInt16MultiArray.h>
#include <vibration_navigator_msgs/WalkingStatus.h>
// Boost
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>

namespace vibration_navigator_driver {

    class ContactTFPublisher
    {
        public:

            ContactTFPublisher(  ros::NodeHandle &nh,
                                 ros::NodeHandle &nh_private,
                                 tf2_ros::Buffer &tf_buffer,
                                 tf2_ros::TransformBroadcaster &tf_broadcaster );

            /**
             * @brief Main loop function. It will start spinner threads for subscribers and start main loop
             * for calculation of commands to vibrators.
             * @param nh ros::NodeHandle ros node handler
             * @param nh_private ros::NodeHandle ros node handler with private namespace
             * @param tf_buffer tf2_ros::Buffer tf2_ros buffer
             */
            void spin();

        private:

            /**
             * ROS
             */
            ros::NodeHandle& nh_;
            ros::NodeHandle& nh_private_;
            tf2_ros::Buffer& tf_buffer_;
            tf2_ros::TransformBroadcaster& tf_broadcaster_;
            ros::Subscriber sub_walkingstatus_;
            std::string reference_frame_id_;
            std::string output_frame_id_;
            std::string fixed_frame_id_;
            ros::Duration duration_timeout_;

            geometry_msgs::TransformStamped transform_msg_;

            int walking_status_;

            bool initialized_;

            /**
             *
             */
            void callbackSub( const vibration_navigator_msgs::WalkingStatus& msg );
    };

}

#endif
