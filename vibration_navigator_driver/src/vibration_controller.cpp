// Standaerd C++ Library
#include <iostream>
#include <vector>
// ROS
#include <ros/ros.h>
#include <ros/spinner.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
// ROS message
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/UInt16MultiArray.h>
// Boost
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>
// Eigen
#include <Eigen/Dense>
// USER
#include "vibration_navigator_driver/vibration_controller.h"

namespace vibration_navigator_driver {

    bool VibrationController::init( ros::NodeHandle &nh, ros::NodeHandle &nh_private, tf2_ros::Buffer &tf_buffer )
    {
        int num_spinthread = 4;
        if ( nh_private.hasParam("num_spinthread") ) {
            nh_private.getParam("num_spinthread", num_spinthread);
        }

        if ( not this->loadConfig( nh, nh_private ) ) {
            return false;
        }

        /**
         *
         */
        this->posestamped_footstep_.header.frame_id = "";

        /*
         * Publisher and Subscriber generation
         */
        this->publisher_commands_ = nh_private.advertise<std_msgs::UInt16MultiArray>("output",10);
        this->subscriber_footstep_ = nh_private.subscribe<geometry_msgs::PoseStamped,VibrationController>(
                "input",
                10,
                &VibrationController::callback,
                this );

        /*
         * ROS spinner generation
         */
        this->ptr_spinner_ = boost::shared_ptr<ros::AsyncSpinner>(new ros::AsyncSpinner(num_spinthread));

        return true;
    }

    void VibrationController::spin( ros::NodeHandle &nh, ros::NodeHandle &nh_private, tf2_ros::Buffer &tf_buffer )
    {
        // start ROS spin thread
        this->ptr_spinner_->start();

        // main loop
        int rate_spin = 10;
        ros::Rate r(rate_spin);
        while ( ros::ok() ) {
            this->updateVibratorCommands( nh, nh_private, tf_buffer );
            this->publishVibratorCommands(); 
            r.sleep();
        }
    }

    bool VibrationController::loadConfig( ros::NodeHandle &nh, ros::NodeHandle &nh_private )
    {
        /*
         * each field of the list in the parameter should be like below.
         *
         * - frame_id: "hoge"
         *   position:
         *     x:
         *     y:
         *     z:
         *   orientation
         *     x:
         *     y
         *     z:
         *     w:
         */
        XmlRpc::XmlRpcValue posesVibrator;
        if ( nh_private.hasParam( "vibrator_config" ) ) {
            nh_private.getParam( "vibrator_config", posesVibrator );
        } else {
            return false;
        }
        ROS_ASSERT( posesVibrator.getType() == XmlRpc::XmlRpcValue::TypeArray );

        for ( int i = 0; i < posesVibrator.size(); i++ ) {
            ROS_ASSERT( posesVibrator[i].getType() == XmlRpc::XmlRpcValue::TypeStruct );
            ROS_ASSERT( posesVibrator[i]["frame_id"].getType() == XmlRpc::XmlRpcValue::TypeString );
            ROS_ASSERT( posesVibrator[i]["position"].getType() == XmlRpc::XmlRpcValue::TypeStruct );
            ROS_ASSERT( posesVibrator[i]["position"]["x"].getType() == XmlRpc::XmlRpcValue::TypeDouble );
            ROS_ASSERT( posesVibrator[i]["position"]["y"].getType() == XmlRpc::XmlRpcValue::TypeDouble );
            ROS_ASSERT( posesVibrator[i]["position"]["z"].getType() == XmlRpc::XmlRpcValue::TypeDouble );
            ROS_ASSERT( posesVibrator[i]["orientation"].getType() == XmlRpc::XmlRpcValue::TypeStruct );
            ROS_ASSERT( posesVibrator[i]["orientation"]["x"].getType() == XmlRpc::XmlRpcValue::TypeDouble );
            ROS_ASSERT( posesVibrator[i]["orientation"]["y"].getType() == XmlRpc::XmlRpcValue::TypeDouble );
            ROS_ASSERT( posesVibrator[i]["orientation"]["z"].getType() == XmlRpc::XmlRpcValue::TypeDouble );
            ROS_ASSERT( posesVibrator[i]["orientation"]["w"].getType() == XmlRpc::XmlRpcValue::TypeDouble );

            Vibrator vib;
            vib.frame_id = static_cast<std::string>(posesVibrator[i]["frame_id"]);
            vib.pose_vibrator.position.x = static_cast<double>(posesVibrator[i]["position"]["x"]);
            vib.pose_vibrator.position.y = static_cast<double>(posesVibrator[i]["position"]["y"]);
            vib.pose_vibrator.position.z = static_cast<double>(posesVibrator[i]["position"]["z"]);
            vib.pose_vibrator.orientation.x = static_cast<double>(posesVibrator[i]["orientation"]["x"]);
            vib.pose_vibrator.orientation.y = static_cast<double>(posesVibrator[i]["orientation"]["y"]);
            vib.pose_vibrator.orientation.z = static_cast<double>(posesVibrator[i]["orientation"]["z"]);
            vib.pose_vibrator.orientation.w = static_cast<double>(posesVibrator[i]["orientation"]["w"]);

            this->vector_vibrators_.push_back( vib );
        }

        return true;
    }

    void VibrationController::callback( geometry_msgs::PoseStamped msg )
    {
         this->posestamped_footstep_ = msg;
    }

    void VibrationController::publishVibratorCommands()
    {
        std_msgs::UInt16MultiArray msg;
        for ( auto itr = this->vector_vibrators_.begin(); itr != this->vector_vibrators_.end(); itr++ ) {
            msg.data.push_back(itr->current_command);
        }
        this->publisher_commands_.publish(msg);
    }

    void VibrationController::updateVibratorCommands(
            ros::NodeHandle &nh,
            ros::NodeHandle &nh_private,
            tf2_ros::Buffer &tf_buffer )
    {
        if ( this->posestamped_footstep_.header.frame_id.length() == 0 ) {
            return;
        }
        for ( auto itr = this->vector_vibrators_.begin(); itr != this->vector_vibrators_.end(); itr++ ) {
            /*
             * posestamped_footstep_ に設定された仮想的な音源位置を元に
             * 各 Vibration オブジェクトが出すべき振動を計算する.
             *
             * このときに,Vibrationオブジェクトの表から振動を受けるのでなければ、
             * 振動を出さないというモデルを考える.
             *
             * また,振動の強さ power は,音源位置からの距離を d として, power = f(d) と表されるとし,
             * f(d) は以下のような形をとるとする.
             *
             *      f(d) = k / d^2 ( k は比例定数 )
             */

            /*
             * 計算を基本的に vibrator の座標系 ( Vibrator.frame_id のフレーム ) で計算するので,
             * Vibrator.frame_id の座標系における, footstepの 位置姿勢 (geometry_msgs::Pose) を求める
             *
             * これは Vibratorが全て同じ frame_id 上にあるのであれば,一回計算すれば良いので,計算速度が遅ければ
             * frame_id が全て同じになるようにする.
             */
            geometry_msgs::TransformStamped tfs_footstep2vibrator;
            try {
            tfs_footstep2vibrator =
                tf_buffer.lookupTransform(
                        itr->frame_id.c_str(),
                        this->posestamped_footstep_.header.frame_id.c_str(),
                        ros::Time(0)
                        );
            } catch (tf2::TransformException &ex) {
                ROS_WARN("%s",ex.what());
                itr->current_command = 0.0;
                continue;
            }
            geometry_msgs::Pose pose_footstep;
            tf2::doTransform( this->posestamped_footstep_.pose, pose_footstep, tfs_footstep2vibrator );
            /*
             * 計算は Eigen のオブジェクトで計算するので, Vibrator.frame_id の座標系における
             * footstep と vibrator の位置姿勢を, Eigen::Vector3d と Eigen::Quaterniond で表す.
             */
            Eigen::Vector3d position_footstep, position_vibrator;
            Eigen::Quaterniond orientation_footstep, orientation_vibrator;
            tf2::fromMsg( static_cast<geometry_msgs::Point>(pose_footstep.position), position_footstep );
            tf2::fromMsg( static_cast<geometry_msgs::Point>(itr->pose_vibrator.position), position_vibrator );
            tf2::fromMsg( static_cast<geometry_msgs::Quaternion>(pose_footstep.orientation), orientation_footstep );
            tf2::fromMsg( static_cast<geometry_msgs::Quaternion>(itr->pose_vibrator.orientation), orientation_vibrator );
            /*
             * 振動の入射方向を計算する.
             * footstep から vibrator への vector と vibrator の姿勢の normal vector との向きを内積で計算して判定する.
             * 内積は, footstep と vibrator との距離に cos(２つのベクトル) をかけたものになる
             */
            Eigen::Vector3d vector_vibrator2footstep = position_footstep - position_vibrator;
            Eigen::Vector3d normal_vibrator = orientation_vibrator.normalized().toRotationMatrix().col(2);
            double d = vector_vibrator2footstep.dot( normal_vibrator );
            double distance = vector_vibrator2footstep.norm();
            if ( d <= 0.0 ) {
                itr->current_command = 0.0;
            } else {
                itr->current_command = (int)this->calcVibrationPower( distance, 0.0, 1.0 );
            }
        }
    }

    double VibrationController::calcVibrationPower( double distance, double theta, double k )
    {
        return k / ( distance * distance );
    }

}
