#ifndef WALKER_LOADS_HPP_
#define WALKER_LOADS_HPP_

#include <functional>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/parameter.hpp>
#include <rclcpp/parameter_map.hpp>

#include <rcl_yaml_param_parser/parser.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "std_msgs/msg/string.hpp"

// Custom Messages related Headers
#include "walker_msgs/msg/step_stamped.hpp"
#include "walker_msgs/msg/force_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"

// Local includes
#include "walker_loads/spline.h"

using std::placeholders::_1;
using namespace std::chrono_literals;

class PartialLoads : public rclcpp::Node{
  public:
      PartialLoads();

  private:
      void loadHandleCalibration();
      void handle_lc(const walker_msgs::msg::ForceStamped::SharedPtr msg);
      void l_steps_lc(const walker_msgs::msg::StepStamped::SharedPtr msg);
      void r_steps_lc(const walker_msgs::msg::StepStamped::SharedPtr msg);
      void user_desc_lc(const std_msgs::msg::String::SharedPtr msg);
      void timer_callback();

      // ROS objects
      rclcpp::Publisher<walker_msgs::msg::StepStamped>::SharedPtr left_load_pub_;
      rclcpp::Publisher<walker_msgs::msg::StepStamped>::SharedPtr right_load_pub_;

      rclcpp::TimerBase::SharedPtr timer_;

      rclcpp::Subscription<walker_msgs::msg::ForceStamped>::SharedPtr left_handle_sub_;
      rclcpp::Subscription<walker_msgs::msg::ForceStamped>::SharedPtr right_handle_sub_;
      rclcpp::Subscription<walker_msgs::msg::StepStamped>::SharedPtr left_steps_sub_;
      rclcpp::Subscription<walker_msgs::msg::StepStamped>::SharedPtr right_steps_sub_;
      rclcpp::Subscription<std_msgs::msg::String>::SharedPtr user_desc_sub_;
 
      //ROS parameters
      std::string handle_calibration_file_;
      std::string scan_topic;
      std::string left_loads_topic_name_;
      std::string right_loads_topic_name_;
      std::string left_handle_topic_name_;
      std::string right_handle_topic_name_;
      std::string left_steps_topic_name_;
      std::string right_steps_topic_name_;
      std::string user_desc_topic_name_;
      int ms_period_;
      double speed_delta_;
 
      // Handle force interpolators
      SplineFunction fl_;
      SplineFunction fr_;

      // Internal state
      walker_msgs::msg::ForceStamped left_handle_msg;
      walker_msgs::msg::ForceStamped right_handle_msg;
      walker_msgs::msg::StepStamped left_step_msg;
      walker_msgs::msg::StepStamped right_step_msg;

      double speed_diff = 0;
      geometry_msgs::msg::Point right_speed;
      geometry_msgs::msg::Point left_speed;

      double weight = 100;
      double right_handle_weight = 0;
      double left_handle_weight  = 0;
      double leg_load = 0;
      bool new_data_available = false;
      bool first_data_ready = false;

};



#endif //WALKER_LOADS_HPP_
