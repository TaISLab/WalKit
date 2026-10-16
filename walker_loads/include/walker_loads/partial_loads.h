#ifndef WALKER_LOADS_HPP_
#define WALKER_LOADS_HPP_

#include <cstdint>
#include <functional>
#include <limits>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/parameter.hpp>
#include <rclcpp/parameter_map.hpp>

#include <rcl_yaml_param_parser/parser.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "std_msgs/msg/header.hpp"

// Custom Messages related Headers
#include "walker_msgs/msg/step_stamped.hpp"
#include "walker_msgs/msg/force_stamped.hpp"
#include "walker_msgs/msg/user_desc.hpp"
#include "geometry_msgs/msg/point.hpp"

// Local includes
#include "walker_loads/spline.h"

#include "walker_loads/diff_tracker.h"
#include "walker_loads/stamp_sequencer.h"

using std::placeholders::_1;
using namespace std::chrono_literals;

// Splits the user's body weight between the two legs and the two handles,
// combining handle force readings (calibrated to kg via SplineFunction)
// with step position/speed (from walker_step_detector) and the user's
// declared weight (/user_desc). A Kalman-filtered left/right speed
// difference (DiffTracker) decides how much of the leg-supported weight
// goes to each leg: fully to one during single-leg stance, split
// proportionally through a smooth ramp during double support.
//
// Everything is driven by the inputs' own stamps (data time), not by a timer
// nor by the clock: the four stamped inputs are put in stamp order
// (StampSequencer) and each one, in that order, updates the filter and
// publishes the loads. The outputs therefore depend only on the messages, not
// on when or in which order they happened to arrive -- the same bag gives the
// same loads on every run, at any replay rate. Cost: a message is processed
// once the slowest input has caught up with it, i.e. at most about one handle
// period (~0.5 s) after it arrived.
//
// Full parameter reference, the Kalman model design and how to reproduce
// this offline against a recorded rosbag: see README.md.
class PartialLoads : public rclcpp::Node{
  public:
      PartialLoads();

  private:
      // The four stamped inputs, in the order that breaks ties between
      // equal stamps
      enum Stream {LEFT_HANDLE = 0, RIGHT_HANDLE = 1, LEFT_STEP = 2, RIGHT_STEP = 3, N_STREAMS = 4};

      struct Input {
          walker_msgs::msg::ForceStamped hand;   // handle message, force already in kg
          walker_msgs::msg::StepStamped step;
      };

      void loadHandleCalibration();
      void l_handle_lc(const walker_msgs::msg::ForceStamped::SharedPtr msg);
      void r_handle_lc(const walker_msgs::msg::ForceStamped::SharedPtr msg);
      void handle_lc(const walker_msgs::msg::ForceStamped::SharedPtr msg, int id);
      void l_steps_lc(const walker_msgs::msg::StepStamped::SharedPtr msg);
      void r_steps_lc(const walker_msgs::msg::StepStamped::SharedPtr msg);
      void user_desc_lc(const walker_msgs::msg::UserDesc::SharedPtr msg);
      void steps_lc(const walker_msgs::msg::StepStamped::SharedPtr msg, int id);
      // Called by the sequencer for each input, in stamp order
      void push_input(int stream, int64_t stamp_ns, Input input);
      void release_inputs();
      void process(int stream, int64_t stamp_ns, Input &input);
      void publish_loads(int64_t stamp_ns);
      bool has_data(int stream, int64_t stamp_ns);
      void update_gait_frequency(double t_ns);

      // ROS objects
      rclcpp::Publisher<walker_msgs::msg::StepStamped>::SharedPtr left_load_pub_;
      rclcpp::Publisher<walker_msgs::msg::StepStamped>::SharedPtr right_load_pub_;
      rclcpp::Publisher<walker_msgs::msg::ForceStamped>::SharedPtr left_hand_load_pub_;
      rclcpp::Publisher<walker_msgs::msg::ForceStamped>::SharedPtr right_hand_load_pub_;

      rclcpp::Subscription<walker_msgs::msg::ForceStamped>::SharedPtr left_handle_sub_;
      rclcpp::Subscription<walker_msgs::msg::ForceStamped>::SharedPtr right_handle_sub_;
      rclcpp::Subscription<walker_msgs::msg::StepStamped>::SharedPtr left_steps_sub_;
      rclcpp::Subscription<walker_msgs::msg::StepStamped>::SharedPtr right_steps_sub_;
      rclcpp::Subscription<walker_msgs::msg::UserDesc>::SharedPtr user_desc_sub_;
 
      //ROS parameters
      std::string handle_calibration_file_;
      std::string scan_topic;
      std::string left_loads_topic_name_;
      std::string right_loads_topic_name_;
      std::string left_hand_loads_topic_name_;
      std::string right_hand_loads_topic_name_;
      std::string left_handle_topic_name_;
      std::string right_handle_topic_name_;
      std::string left_steps_topic_name_;
      std::string right_steps_topic_name_;
      std::string user_desc_topic_name_;
      double speed_delta_;
      double speed_delta_ratio_;
      double speed_delta_max_;
      double data_timeout_s_;
      // how long (data time) the sequencer waits for a silent input before
      // giving up on it, see StampSequencer
      double max_wait_s_;
      // plausible range for the gait angular frequency measured from
      // step-alternation timing (see update_gait_frequency()); estimates
      // outside this range are dropped rather than fed to the Kalman tracker
      double w_estimate_min_;
      double w_estimate_max_;
      // minimum time between accepted zero-crossings of speed_diff_, to
      // reject crossings caused by sensor noise near zero rather than an
      // actual swing/stance role swap
      double zero_crossing_refractory_s_;
      bool debug_output_ = false;
 
      // Handle force interpolators
      SplineFunction fl_;
      SplineFunction fr_;

      // Internal state
      // as provided by sensor
      walker_msgs::msg::ForceStamped left_handle_msg_;
      walker_msgs::msg::ForceStamped right_handle_msg_;
      // data in kg
      walker_msgs::msg::ForceStamped left_hand_msg_;
      walker_msgs::msg::ForceStamped right_hand_msg_;
      walker_msgs::msg::StepStamped left_step_msg_;
      walker_msgs::msg::StepStamped right_step_msg_;

      double speed_diff_ = 0;
      geometry_msgs::msg::Point right_speed_;
      geometry_msgs::msg::Point left_speed_;

      // zero-crossing detector state, used to measure the gait's angular
      // frequency (w) directly from step-alternation timing instead of
      // leaving it to the Kalman tracker's own weak, indirect observation
      // of it (see update_gait_frequency())
      double prev_speed_diff_for_zc_ = 0.0;
      bool prev_speed_diff_valid_ = false;
      double last_crossing_time_s_ = 0.0;
      bool has_last_crossing_ = false;

      // stamp (ns) of the last message processed of each input, used to detect
      // stale data: kNoData until the first one
      static constexpr int64_t kNoData = std::numeric_limits<int64_t>::min();
      int64_t last_stamp_ns_[N_STREAMS] = {kNoData, kNoData, kNoData, kNoData};
      int64_t data_timeout_ns_ = 0;

      // The loads are published on a fixed grid of data time (ms_period), the
      // stamp-driven stand-in for a timer: the state as of each tick, if
      // anything new arrived since the previous one
      int ms_period_;
      int64_t period_ns_ = 0;
      int64_t next_tick_ns_ = 0;
      bool have_tick_ = false;
      bool new_data_available_ = false;
      std::unique_ptr<StampSequencer<Input>> sequencer_;

      int weight_ = 100;
      // /user_desc is a one-shot, transient_local config value (like a
      // parameter), not a continuous stream: once received it doesn't need
      // to keep arriving, so it's a permanent latch, unlike the rx_time_
      // members above.
      bool weight_received_ = false;
      double leg_load_ = 0;
      bool first_data_ready_ = false;

      DiffTracker kalman_tracker_;
};

#endif //WALKER_LOADS_HPP_
