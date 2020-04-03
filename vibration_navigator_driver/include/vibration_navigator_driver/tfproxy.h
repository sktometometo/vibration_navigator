#ifndef VIBRATION_NAVIGATOR_TFPROXY_H__
#define VIBRATION_NAVIGATOR_TFPROXY_H__

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

namespace vibration_navigator_driver {

    /**
     * @brief The TFProxy class is for calculate and publish walking status of each foot
     */
    class TFProxy
    {
        public:

            /**
             * @brief Initialization function. Please call this function before to start spin() function.
             * @param nh ros::NodeHandle ros node handler
             * @param nh_private ros::NodeHandle ros node handler with private namespace
             * @param tf_buffer tf2_ros::Buffer tf2_ros buffer
             */
            bool init( ros::NodeHandle &nh,
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
            void spin( ros::NodeHandle &nh,
                       ros::NodeHandle &nh_private,
                       tf2_ros::Buffer &tf_buffer,
                       tf2_ros::TransformBroadcaster &tf_broadcaster );

        private:

            /**
             * ROS
             */
            ros::Publisher publisher_walking_status_;

            std::string reference_frame_id_;
            std::string output_frame_id_;
            std::string fixed_frame_id_;

            bool initialized_;

            geometry_msgs::TransformStamped msg_transform_;

            /**
             *
             */
            void TFProxy::callbackTimerTF(
                                            tf2_ros::Buffer &tf_buffer,
                                            tf2_ros::TransformBroadcaster &tf_broadcaster,
                                            const ros::TimerEvent& );
    };

}
