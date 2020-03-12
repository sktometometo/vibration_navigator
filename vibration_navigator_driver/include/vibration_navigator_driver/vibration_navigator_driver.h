#ifndef VIBRATION_NAVIGATOR_DRIVER_H__
#define VIBRATION_NAVIGATOR_DRIVER_H__

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
// Boost
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>

namespace vibration_navigator_driver {

    class Vibrator
    {
        public:

            /**
             * Information abouc position of a vibrator
             */
            std::string frame_id; // frame_id to which this vibrator is attached.
            geometry_msgs::Pose pose_vibrator; // pose in attaced frame_id

            /**
             * Information about controlling of vibrator
             */
            int current_command;
    };

    class VibrationNavigatorDriver
    {
        public:

            /**
             * initialization function
             * @param nh ros node handler
             * @param nh_private ros node handler with private namespace
             * @param tf_buffer tf2_ros buffer
             */
            bool init( ros::NodeHandle &nh, ros::NodeHandle &nh_private, tf2_ros::Buffer &tf_buffer );

            /**
             * main loop function
             * @param nh ros node handler
             * @param nh_private ros node handler with private namespace
             * @param tf_buffer tf2_ros buffer
             */
            void spin( ros::NodeHandle &nh, ros::NodeHandle &nh_private, tf2_ros::Buffer &tf_buffer );

            /**
             * this function will load pose configuration of vibrators from parameter server.
             * @param nh ros node handler
             * @param nh_private ros node handler with private namespace
             */
            bool loadConfig( ros::NodeHandle &nh, ros::NodeHandle &nh_private );

        private:

            /**
             * member values
             */
            std::vector<Vibrator> vector_vibrators_;
            geometry_msgs::PoseStamped posestamped_footstep_; // 設定された音源位置を示す PoseStamped オブジェクト
            bool is_set_target_; // 音源位置が設定されているかどうか
            // ROS
            ros::Publisher publisher_commands_;
            ros::Subscriber subscriber_footstep_;
            boost::shared_ptr<ros::AsyncSpinner> ptr_spinner_;

            /**
             * ROS callback function for pose stamped message representing the next footstep.
             */
            void callback( geometry_msgs::PoseStamped msg );

            /**
             * Publish a command array to vibrators
             */
            void publishVibratorCommands();

            /**
             * Calculate and update the commands to vibrators with transformations of each frames
             * @param nh ros node handler
             * @param nh_private ros node handler with private namespace
             * @param tf_buffer tf2_ros buffer
             */
            void updateVibratorCommands(
                        ros::NodeHandle &nh,
                        ros::NodeHandle &nh_private,
                        tf2_ros::Buffer &tf_buffer );

            /**
             * Calculate power of a vibrator
             * @param distance distance from footstep to vibrator [m]
             * @param theta the angle between a vector from vibrator to footstep and a normal vector of the vibrator orientation. [radians]
             * @param k a constant
             */
            double calcVibrationPower( double distance, double theta, double k );
    };
}

#endif
