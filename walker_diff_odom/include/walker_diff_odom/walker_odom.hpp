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
#include "walker_stamp_utils/stamp_sequencer.h"

using std::placeholders::_1;
using namespace std;
using namespace Eigen;

// Wheel odometry on data time: no timer and no clock. The two encoder streams go through a StampSequencer (so
// the left and right counts used together are those of the same instant, whatever order the topics arrive in)
// and the odometry is published on a fixed grid of data time (tf_rate_hz), with the state as of each tick, if a
// wheel moved since the previous one. Stamps and velocities come from that grid: the same encoder messages give
// the same odometry at any replay rate.
class WalkerDiffDrive : public rclcpp::Node{
  public:
      WalkerDiffDrive();

  private:
      enum Stream {LEFT = 0, RIGHT = 1};

      void leftEncoderCallback(const walker_msgs::msg::EncoderStamped::ConstSharedPtr left_enc_msg) ;
    
      void rightEncoderCallback(const walker_msgs::msg::EncoderStamped::ConstSharedPtr right_enc_msg) ;

      // In stamp order, from the sequencer
      void processEncoder(int stream, int64_t stamp_ns, const walker_msgs::msg::EncoderStamped &msg);
      void leftEncoder(const walker_msgs::msg::EncoderStamped &msg);
      void rightEncoder(const walker_msgs::msg::EncoderStamped &msg);
      
      void update(int64_t stamp_ns);

      // ROS objects
      rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
      std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

      rclcpp::Subscription<walker_msgs::msg::EncoderStamped>::SharedPtr sub_left_;
      rclcpp::Subscription<walker_msgs::msg::EncoderStamped>::SharedPtr sub_right_;

      //parameters
      bool publish_tf_;
      double rate_hz_;
      double max_wait_s_;
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

      // data-time grid the odometry is published on, and the time of the previous publication
      std::unique_ptr<StampSequencer<walker_msgs::msg::EncoderStamped>> sequencer_;
      int64_t period_ns_ = 0;
      int64_t next_tick_ns_ = 0;
      bool have_tick_ = false;
      int64_t then_ns_ = 0;
      bool have_then_ = false;
      geometry_msgs::msg::TransformStamped transform_;
      nav_msgs::msg::Odometry odom_;
      walker_msgs::msg::EncoderStamped last_left_data_;
      walker_msgs::msg::EncoderStamped last_right_data_;
};



#endif //WALKER_ODOM_HPP_
