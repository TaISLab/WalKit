#ifndef WALKER_ODOM_HPP_
#define WALKER_ODOM_HPP_

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cmath>
#include <eigen3/Eigen/Core>
#include <iostream>

#include "rclcpp/rclcpp.hpp"

#include <tf2_ros/transform_broadcaster.h>

#include "walker_msgs/msg/encoder_stamped.hpp"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <nav_msgs/msg/odometry.hpp>

#include "std_msgs/msg/string.hpp"

#include "walker_diff_odom/circular_buffer.hpp"

using std::placeholders::_1;
using namespace std;
using namespace Eigen;

class WalkerDiffDrive : public rclcpp::Node{
  public:
      WalkerDiffDrive();

  private:
      void leftEncoderCallback(const walker_msgs::msg::EncoderStamped::ConstSharedPtr left_enc_msg) ;
    
      void rightEncoderCallback(const walker_msgs::msg::EncoderStamped::ConstSharedPtr right_enc_msg) ;
      
      void update();

      // ROS objects
      rclcpp::TimerBase::SharedPtr update_timer_;
      rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
      std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

      rclcpp::Subscription<walker_msgs::msg::EncoderStamped>::SharedPtr sub_left_;
      rclcpp::Subscription<walker_msgs::msg::EncoderStamped>::SharedPtr sub_right_;

      //parameters
      bool publish_tf_;
      double rate_hz_;
      std::string odom_topic_name_;
      std::string left_wheel_encoder_topic_name_;
      std::string right_wheel_encoder_topic_name_;

      double ticks_meter_;
      double base_width_;
      std::string base_frame_id_;
      std::string odom_frame_id_;
      int encoder_min_;
      int encoder_max_;
      int encoder_low_wrap_;
      int encoder_high_wrap_;

      // internal data
      CircularBuffer left_buffer_;
      CircularBuffer right_buffer_;
      
      bool first_reading_;
      bool new_left_;
      bool new_right_;
      double enc_left_;
      double enc_right_;
      double left_;
      double right_;
      double lmult_;
      double rmult_;
      double prev_lencoder_;
      double prev_rencoder_;
      double x_;
      double y_;
      double th_;
      double dx_;
      double dr_;
      double kr ;   // noise factor in right wheel   
      double kl ;   // noise factor in left wheel  

      // matrixes used in covariance calculus
      // Ed wheel covariance 
      Matrix<float, 2, 2> Ed; 
      // Fp position Jacobian
      Matrix<float, 3, 3> Fp;
      // Fd wheel - position Jacobian      
      Matrix<float, 3, 2> Fd;
      // Ep position covariance
      Matrix<float, 3, 3> Ep;

      rclcpp::Time then_;
      rclcpp::Time now_;
      geometry_msgs::msg::TransformStamped transform_;
      nav_msgs::msg::Odometry odom_;
      walker_msgs::msg::EncoderStamped last_left_data_;
      walker_msgs::msg::EncoderStamped last_right_data_;
};



#endif //WALKER_ODOM_HPP_
