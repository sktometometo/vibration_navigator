// ROS
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
// USER
#include "vibration_navigator_driver/tfproxy.h"

int main( int argc, char** argv )
{
    //
    ros::init( argc, argv, "tfproxy_node" );
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    //
    double max_duration;
    nh_private.param<double>("max_duration", max_duration, 5.0);

    //
    ros::Duration tfduration(max_duration);
    tf2_ros::Buffer            tf_buffer(tfduration);
    tf2_ros::TransformListener tf_listener(tf_buffer);
    tf2_ros::TransformBroadcaster tf_broadcaster;

    //
    vibration_navigator_driver::TFProxy tfproxy;
    tfproxy.init( nh, nh_private, tf_buffer, tf_broadcaster );
    tfproxy.spin( nh, nh_private, tf_buffer, tf_broadcaster );
}
