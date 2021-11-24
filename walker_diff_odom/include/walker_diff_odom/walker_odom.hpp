#ifndef WALKER_ODOM_HPP_
#define WALKER_ODOM_HPP_

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <tf2_ros/transform_broadcaster.h>

#include "walker_msgs/msg/encoder_stamped.hpp"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <nav_msgs/msg/odometry.hpp>


using namespace message_filters::sync_policies;
using namespace std::placeholders;
using namespace std::chrono_literals;

class WalkerDiffDrive : public rclcpp::Node{
  public:
      WalkerDiffDrive();

  private:
      void encoderCallback(const walker_msgs::msg::EncoderStamped::ConstSharedPtr &left_enc_msg,
                           const walker_msgs::msg::EncoderStamped::ConstSharedPtr &right_enc_msg);

      void update();

      void declare_and_get_parameter(std::string param_name, rclcpp::ParameterValue param_value, std::string param_var);

      void declare_and_get_parameter(std::string param_name, rclcpp::ParameterValue param_value, int param_var);

      void declare_and_get_parameter(std::string param_name, rclcpp::ParameterValue param_value, double param_var);

      // ROS objects
      rclcpp::TimerBase::SharedPtr update_timer_;
      rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
      std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

      using SyncPolicy = message_filters::sync_policies::ApproximateTime<walker_msgs::msg::EncoderStamped, walker_msgs::msg::EncoderStamped>;
      using Synchronizer = message_filters::Synchronizer<SyncPolicy>;
      std::shared_ptr<Synchronizer> sync_;
      message_filters::Subscriber<walker_msgs::msg::EncoderStamped> sub_left_;
      message_filters::Subscriber<walker_msgs::msg::EncoderStamped> sub_right_;

      //parameters
      double rate_hz_;
      std::string odom_topic_name_;
      std::string left_wheel_encoder_topic_name_;
      std::string right_wheel_encoder_topic_name_;

      int ticks_meter_;
      double base_width_;
      std::string base_frame_id_;
      std::string odom_frame_id_;
      int encoder_min_;
      int encoder_max_;
      int encoder_low_wrap_;
      int encoder_high_wrap_;

      // internal data
      bool first_reading_;
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
      rclcpp::Time then_;
      rclcpp::Time now_;
      geometry_msgs::msg::TransformStamped transform_;
      nav_msgs::msg::Odometry odom_;
};



#endif //WALKER_ODOM_HPP_
