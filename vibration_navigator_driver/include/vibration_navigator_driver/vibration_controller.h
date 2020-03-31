#ifndef VIBRATION_NAVIGATOR_DRIVER_VIBRATION_CONTROLLER_H__
#define VIBRATION_NAVIGATOR_DRIVER_VIBRATION_CONTROLLER_H__

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

    /**
     * @brief The Vibrator class is for storing information about each vibrator.
     */
    class Vibrator
    {
        public:

            /**
             * Information abouc position of a vibrator
             */
            std::string frame_id; /* frame_id to which this vibrator is attached. */
            geometry_msgs::Pose pose_vibrator; /* pose of a vibrator in the attaced frame */

            /**
             * Information about controlling of vibrator
             */
            int current_command;
    };


    /**
     * @brief The VibrationController class is a wrapper node to calculate commands of each vibrator
     * from a given target footstep pose.
     */
    class VibrationController
    {
        public:

            /**
             * @brief Initialization function. Please call this function before to start spin() function.
             * @param nh ros::NodeHandle ros node handler
             * @param nh_private ros::NodeHandle ros node handler with private namespace
             * @param tf_buffer tf2_ros::Buffer tf2_ros buffer
             */
            bool init( ros::NodeHandle &nh, ros::NodeHandle &nh_private, tf2_ros::Buffer &tf_buffer );

            /**
             * @brief Main loop function. It will start spinner threads for subscribers and start main loop
             * for calculation of commands to vibrators.
             * @param nh ros::NodeHandle ros node handler
             * @param nh_private ros::NodeHandle ros node handler with private namespace
             * @param tf_buffer tf2_ros::Buffer tf2_ros buffer
             */
            void spin( ros::NodeHandle &nh, ros::NodeHandle &nh_private, tf2_ros::Buffer &tf_buffer );

        private:

            /**
             * member values
             */
            std::vector<Vibrator> vector_vibrators_; /* Vibrator の情報を格納するvector */
            geometry_msgs::PoseStamped posestamped_footstep_; /* 設定されたfootstepの位置姿勢を示す PoseStamped オブジェクト */
            bool is_set_target_; /* footstep の位置姿勢が設定されているかどうかを示す Flag */
            // ROS
            ros::Publisher publisher_commands_;
            ros::Subscriber subscriber_footstep_;
            boost::shared_ptr<ros::AsyncSpinner> ptr_spinner_;

            /**
             * @brief this function will load pose configuration of vibrators from parameter server.
             * @param nh ros::NodeHandle ros node handler
             * @param nh_private ros::NodeHandle ros node handler with private namespace
             */
            bool loadConfig( ros::NodeHandle &nh, ros::NodeHandle &nh_private );

            /**
             * @brief ROS callback function for pose stamped message representing the next footstep.
             * @param msg geometry_msgs::PoseStamped subscribed message
             */
            void callback( geometry_msgs::PoseStamped msg );

            /**
             * @brief Publish a command array to vibrators
             */
            void publishVibratorCommands();

            /**
             * @brief Calculate and update the commands to vibrators with transformations of each frames
             * @param nh ros::NodeHandle ros node handler
             * @param nh_private ros::NodeHandle ros node handler with private namespace
             * @param tf_buffer tf2_ros::Buffer tf2_ros buffer
             */
            void updateVibratorCommands(
                        ros::NodeHandle &nh,
                        ros::NodeHandle &nh_private,
                        tf2_ros::Buffer &tf_buffer );

            /**
             * @brief Calculate power of a vibrator
             * @param distance double distance from footstep to vibrator [m]
             * @param theta double the angle between a vector from vibrator to footstep and a normal vector of the vibrator orientation. [radians]
             * @param k double a constant
             */
            double calcVibrationPower( double distance, double theta, double k );
    };
}

#endif
