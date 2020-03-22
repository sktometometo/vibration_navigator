// ROS
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
// USER
#include "vibration_navigator_driver/vibration_navigator_driver.h"

int main( int argc, char** argv )
{
    //
    ros::init( argc, argv, "vibration_navigator_driver_node" );
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    //
    double max_duration;
    nh_private.param<double>("max_duration", max_duration, 5.0);

    //
    tf2_ros::Buffer            tf_buffer(ros::Duration(max_duration*2));
    tf2_ros::TransformListener tf_listener(tf_buffer);

    //
    vibration_navigator_driver::VibrationNavigatorDriver driver;
    bool ret = driver.init( nh, nh_private, tf_buffer );
    if ( ret ) {
        driver.spin( nh, nh_private, tf_buffer );
    } else {
        ROS_ERROR( "Initialization failed." );
    }
}
