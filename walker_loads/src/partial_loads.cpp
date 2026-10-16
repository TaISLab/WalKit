#include "walker_loads/partial_loads.h"


PartialLoads::PartialLoads() : Node("partial_loads"){
        handle_calibration_file_ = ament_index_cpp::get_package_share_directory("walker_loads") + "/config/handle_calib.yaml";

        //Declare ROS parameters
        this->declare_parameter<std::string>("scan_topic",              "/scan");
        this->declare_parameter<std::string>("left_loads_topic_name",   "/left_loads");
        this->declare_parameter<std::string>("right_loads_topic_name",  "/right_loads");
        this->declare_parameter<std::string>("left_hand_loads_topic_name",   "/left_hand_loads");
        this->declare_parameter<std::string>("right_hand_loads_topic_name",  "/right_hand_loads");
        this->declare_parameter<std::string>("left_handle_topic_name",  "/left_handle");
        this->declare_parameter<std::string>("right_handle_topic_name", "/right_handle");
        this->declare_parameter<std::string>("left_steps_topic_name",   "/detected_step_left");
        this->declare_parameter<std::string>("right_steps_topic_name",  "/detected_step_right");
        this->declare_parameter<std::string>("user_desc_topic_name",    "/user_desc");
        this->declare_parameter<std::string>("handle_calibration_file", handle_calibration_file_);
        this->declare_parameter<double>("speed_delta",                  0.05);
        this->declare_parameter<double>("speed_delta_ratio",            0.3);
        this->declare_parameter<double>("speed_delta_max",              1.0);
        // Period (ms) of the grid, on data time, the loads are sampled and
        // published on: with the state as of each tick, if anything new
        // arrived since the previous one. The gait parameters counted from
        // them (step loads = a leg's load going from 0 to positive) depend on
        // it: the sampling is what filters the chatter of the leg-load ramp.
        this->declare_parameter<int>("ms_period",                       50);
        // An input silent for longer than this (data time: the stamps of
        // the other inputs, not the clock) is treated as no data, and the
        // loads stop being published until it comes back.
        this->declare_parameter<double>("data_timeout_s",               1.0);
        // How long (data time) inputs are held back waiting for a silent one
        // before it is given up on, see StampSequencer. Messages are
        // normally released within one handle period (~0.5 s) of arriving;
        // this only bounds the wait when an input has stopped altogether. It
        // must be longer than the largest lag between inputs (replaying at
        // --rate N, N times the processing delay), or the outputs depend on
        // timing again.
        this->declare_parameter<double>("max_wait_s",                   5.0);
        // Plausible range for the gait angular frequency measured from
        // step-alternation timing (a zero-crossing detector on speed_diff_,
        // see update_gait_frequency()); a rollator-assisted gait cycle is
        // unlikely to be faster than ~0.8s or slower than ~20s.
        this->declare_parameter<double>("w_estimate_min",               0.3);   // rad/s (~20s/cycle)
        this->declare_parameter<double>("w_estimate_max",               8.0);   // rad/s (~0.8s/cycle)
        this->declare_parameter<double>("zero_crossing_refractory_s",   0.2);
        this->declare_parameter<bool>("debug_output",                   false);
        // Kalman diff-tracker priors (initial state, before any measurement
        // arrives). (kalman_va,kalman_vb)/(kalman_fa,kalman_fb) are the
        // Cartesian in-phase/quadrature components of the speed-diff/
        // force-diff oscillations (see kalman/DiffSystemModel.hpp) -- there
        // is no separate delay parameter any more, it's implicit in how the
        // force pair relates to the speed pair. kalman_w's default was
        // 0.1 rad/s (~63s per gait cycle, implausible for walking) -- 2.0
        // rad/s corresponds to a ~3s cycle, a more reasonable starting point
        // for a rollator-assisted gait. These were previously hardcoded and
        // could only be changed by recompiling.
        this->declare_parameter<double>("kalman_v0",                    0.01);  // m/s
        this->declare_parameter<double>("kalman_va",                    1.0);   // m/s
        this->declare_parameter<double>("kalman_vb",                    0.0);   // m/s
        this->declare_parameter<double>("kalman_f0",                    4.0);   // kg
        this->declare_parameter<double>("kalman_fa",                    15.0);  // kg
        this->declare_parameter<double>("kalman_fb",                    0.0);   // kg
        this->declare_parameter<double>("kalman_w",                     2.0);   // rad/s
        this->declare_parameter<double>("kalman_theta",                 0.20);  // rad

        //Get ROS parameters
        this->get_parameter("scan_topic",              scan_topic);
        this->get_parameter("left_loads_topic_name",   left_loads_topic_name_);
        this->get_parameter("right_loads_topic_name",  right_loads_topic_name_);
        this->get_parameter("left_hand_loads_topic_name",   left_hand_loads_topic_name_);
        this->get_parameter("right_hand_loads_topic_name",  right_hand_loads_topic_name_);
        this->get_parameter("left_handle_topic_name",  left_handle_topic_name_);
        this->get_parameter("right_handle_topic_name", right_handle_topic_name_);
        this->get_parameter("left_steps_topic_name",   left_steps_topic_name_);
        this->get_parameter("right_steps_topic_name",  right_steps_topic_name_);
        this->get_parameter("user_desc_topic_name",    user_desc_topic_name_);
        this->get_parameter("handle_calibration_file", handle_calibration_file_);
        this->get_parameter("speed_delta",             speed_delta_);
        this->get_parameter("speed_delta_ratio",       speed_delta_ratio_);
        this->get_parameter("speed_delta_max",         speed_delta_max_);
        this->get_parameter("ms_period",               ms_period_);
        this->get_parameter("data_timeout_s",          data_timeout_s_);
        this->get_parameter("max_wait_s",              max_wait_s_);
        this->get_parameter("w_estimate_min",          w_estimate_min_);
        this->get_parameter("w_estimate_max",          w_estimate_max_);
        this->get_parameter("zero_crossing_refractory_s", zero_crossing_refractory_s_);
        this->get_parameter("debug_output",            debug_output_);
        double kalman_v0, kalman_va, kalman_vb, kalman_f0, kalman_fa, kalman_fb, kalman_w, kalman_theta;
        this->get_parameter("kalman_v0",               kalman_v0);
        this->get_parameter("kalman_va",               kalman_va);
        this->get_parameter("kalman_vb",               kalman_vb);
        this->get_parameter("kalman_f0",               kalman_f0);
        this->get_parameter("kalman_fa",               kalman_fa);
        this->get_parameter("kalman_fb",               kalman_fb);
        this->get_parameter("kalman_w",                kalman_w);
        this->get_parameter("kalman_theta",             kalman_theta);

        //Load config files
        loadHandleCalibration();

        // Internal state data
        speed_diff_ = 0;
        weight_ = 100;
        weight_received_ = false;
        right_hand_msg_.force = 0;
        left_hand_msg_.force  = 0;
        leg_load_ = 0;
        first_data_ready_ = false;

        data_timeout_ns_ = static_cast<int64_t>(data_timeout_s_ * 1e9);
        period_ns_ = static_cast<int64_t>(ms_period_) * 1000000;
        sequencer_ = std::make_unique<StampSequencer<Input>>(
            N_STREAMS, static_cast<int64_t>(max_wait_s_ * 1e9), 100000,
            [this](int stream, int64_t stamp_ns, Input &input){ process(stream, stamp_ns, input); });


        // Load kalman tracker
        kalman_tracker_.init(this, "diff_tracker", kalman_v0, kalman_va, kalman_vb,
                              kalman_f0, kalman_fa, kalman_fb, kalman_w, kalman_theta,
                              w_estimate_min_, w_estimate_max_);

        // Set debug output
        if (debug_output_){
            kalman_tracker_.enable_log();

            auto ret = rcutils_logging_set_logger_level( this->get_logger().get_name(), RCUTILS_LOG_SEVERITY_DEBUG);
            if (ret != RCUTILS_RET_OK) {
                RCLCPP_ERROR(this->get_logger(), "Error setting severity: %s", rcutils_get_error_string().str);
                rcutils_reset_error();
            }

        } else {
            RCLCPP_INFO(this->get_logger(), "Partial loads loading.");
        }

        // ROS Comms
        // Deep queue: inputs wait in the sequencer for the slowest one, and a
        // replayed bag can deliver them in bursts; losing one would make the
        // result depend on timing again.
        auto default_qos = rclcpp::QoS(rclcpp::KeepLast(500));
        // user_desc is published once, latched (transient_local), same as
        // every other subscriber to this topic in the codebase (plot_loads.py,
        // gait_monitor_handles.py, walker_data_csv.py, walker_stability.py).
        // The default (volatile) QoS used below for the rest of the topics is
        // compatible but does NOT replay the retained sample to a
        // late-joining subscriber, so a node that starts after user_desc was
        // published would otherwise never see it.
        auto user_desc_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();

        // Publishers. Deep queues: releasing the inputs in stamp order makes
        // the outputs bursty (everything the slowest input was holding back
        // comes out at once), and a reliable writer with a short history
        // overwrites what a slower subscriber has not taken yet.
        auto out_qos = rclcpp::QoS(rclcpp::KeepLast(1000));
        left_load_pub_  = this->create_publisher<walker_msgs::msg::StepStamped>(left_loads_topic_name_, out_qos);
        right_load_pub_ = this->create_publisher<walker_msgs::msg::StepStamped>( right_loads_topic_name_, out_qos);
        left_hand_load_pub_  = this->create_publisher<walker_msgs::msg::ForceStamped>(left_hand_loads_topic_name_, out_qos);
        right_hand_load_pub_ = this->create_publisher<walker_msgs::msg::ForceStamped>( right_hand_loads_topic_name_, out_qos);

        // Subscribers
        left_handle_sub_ = this->create_subscription<walker_msgs::msg::ForceStamped>( left_handle_topic_name_, default_qos, std::bind(&PartialLoads::l_handle_lc, this, std::placeholders::_1));
        right_handle_sub_ = this->create_subscription<walker_msgs::msg::ForceStamped>( right_handle_topic_name_, default_qos, std::bind(&PartialLoads::r_handle_lc, this, std::placeholders::_1));
        left_steps_sub_ = this->create_subscription<walker_msgs::msg::StepStamped>( left_steps_topic_name_, default_qos, std::bind(&PartialLoads::l_steps_lc, this, std::placeholders::_1));
        right_steps_sub_ = this->create_subscription<walker_msgs::msg::StepStamped>( right_steps_topic_name_, default_qos, std::bind(&PartialLoads::r_steps_lc, this, std::placeholders::_1));
        user_desc_sub_ = this->create_subscription<walker_msgs::msg::UserDesc>( user_desc_topic_name_, user_desc_qos, std::bind(&PartialLoads::user_desc_lc, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "CPP load detector started");
    }

    void PartialLoads::loadHandleCalibration(){

        rcl_params_t * yaml_params = rcl_yaml_node_struct_init(rcl_get_default_allocator());
        if (!rcl_parse_yaml_file(handle_calibration_file_.c_str(), yaml_params)){
            throw std::runtime_error("Failed to load calibration data from " + handle_calibration_file_ + "(" + rcl_get_error_string().str + ")");
        }

        rclcpp::ParameterMap param_map = rclcpp::parameter_map_from(yaml_params);
        rcl_yaml_node_struct_fini(yaml_params);

        rclcpp::ParameterMap::iterator it;

		std::vector<double> left_handle_points_ ;
		std::vector<double> right_handle_points_ ;
		std::vector<double> weight_points_ ;

        for (it = param_map.begin(); it != param_map.end(); it++) {
            std::string node_name(it->first.substr(1));
            RCLCPP_INFO(this->get_logger(), "Node name [%s] ", node_name.c_str());			
            for (auto & param : it->second){
                std::string param_name(param.get_name());

                // exact match on the known calibration keys: a substring
                // match here would silently misassign a future key that
                // merely contains "left"/"right"/"weight" (e.g. "weight_scale")
                if (param_name == "left_handle_points"){
					left_handle_points_  = param.get_value<std::vector<double>>();
				}

                if (param_name == "right_handle_points"){
					right_handle_points_  = param.get_value<std::vector<double>>();
				}

                if (param_name == "weight_points"){
					weight_points_  = param.get_value<std::vector<double>>();
				}

            }
        }

        Eigen::VectorXd left_handle_points = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(left_handle_points_.data(), left_handle_points_.size());
        Eigen::VectorXd right_handle_points = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(right_handle_points_.data(), right_handle_points_.size());
		Eigen::VectorXd weight_points = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(weight_points_.data(), weight_points_.size());

        // We will use these to cast from readings to force.
		fl_ = SplineFunction(left_handle_points, weight_points);
        fr_ = SplineFunction(right_handle_points, weight_points);
        
    }

    void PartialLoads::l_handle_lc(const walker_msgs::msg::ForceStamped::SharedPtr msg)  {
        handle_lc(msg, 0);
    }

    void PartialLoads::r_handle_lc(const walker_msgs::msg::ForceStamped::SharedPtr msg)  {
        handle_lc(msg, 1);
    }

    void PartialLoads::handle_lc(const walker_msgs::msg::ForceStamped::SharedPtr msg, int id)  {
        if (id != 0 && id != 1){
            RCLCPP_ERROR(this->get_logger(), "Don't know about which handle are you talking [%d]", id);
            return;
        }

        // The calibrated hand load depends on this message alone, so it goes
        // out right away; what depends on other inputs waits in the sequencer.
        Input input;
        input.hand = *msg;
        input.hand.force = (id == 1 ? fr_ : fl_).interp(msg->force);
        (id == 1 ? right_hand_load_pub_ : left_hand_load_pub_)->publish(input.hand);

        int stream = (id == 1 ? RIGHT_HANDLE : LEFT_HANDLE);
        push_input(stream, rclcpp::Time(msg->header.stamp).nanoseconds(), std::move(input));
    }

    void PartialLoads::l_steps_lc(const walker_msgs::msg::StepStamped::SharedPtr msg)  {
        steps_lc(msg, 0);
    }

    void PartialLoads::r_steps_lc(const walker_msgs::msg::StepStamped::SharedPtr msg)  {
        steps_lc(msg, 1);
    }

    void PartialLoads::steps_lc(const walker_msgs::msg::StepStamped::SharedPtr msg, int id){
        if (id != 0 && id != 1){
            RCLCPP_ERROR(this->get_logger(), "Don't know about which step are you talking [%d]", id);
            return;
        }

        Input input;
        input.step = *msg;
        int stream = (id == 1 ? RIGHT_STEP : LEFT_STEP);
        push_input(stream, rclcpp::Time(msg->position.header.stamp).nanoseconds(), std::move(input));
    }

    void PartialLoads::push_input(int stream, int64_t stamp_ns, Input input){
        if (!sequencer_->push(stream, stamp_ns, std::move(input)) && (sequencer_->late() % 100) == 1){
            RCLCPP_WARN(this->get_logger(),
                "Dropped %lu inputs older than what was already processed (an input stayed silent for more than max_wait_s)",
                static_cast<unsigned long>(sequencer_->late()));
        }
        release_inputs();
    }

    void PartialLoads::release_inputs(){
        // nothing is processed before the user's weight is known: the
        // first load must not depend on whether /user_desc beat the data
        if (weight_received_){
            sequencer_->release();
        }
    }

    void PartialLoads::process(int stream, int64_t stamp_ns, Input &input){
        // the ticks that fall before this input see the state without it
        if (!have_tick_){
            next_tick_ns_ = stamp_ns + period_ns_;
            have_tick_ = true;
        }
        while (next_tick_ns_ < stamp_ns){
            publish_loads(next_tick_ns_);
            next_tick_ns_ += period_ns_;
        }

        last_stamp_ns_[stream] = stamp_ns;
        new_data_available_ = true;

        switch (stream){
            case LEFT_HANDLE:
            case RIGHT_HANDLE:
                if (stream == LEFT_HANDLE){
                    left_hand_msg_ = input.hand;
                } else {
                    right_hand_msg_ = input.hand;
                }
                if (has_data(LEFT_HANDLE, stamp_ns) && has_data(RIGHT_HANDLE, stamp_ns)){
                    double force_diff = left_hand_msg_.force - right_hand_msg_.force;
                    // Kalman this!
                    kalman_tracker_.add_force_measurement(force_diff, static_cast<double>(stamp_ns));
                }
                break;

            case LEFT_STEP:
            case RIGHT_STEP:
                if (stream == LEFT_STEP){
                    left_step_msg_ = input.step;
                    left_speed_ = left_step_msg_.speed;
                } else {
                    right_step_msg_ = input.step;
                    right_speed_ = right_step_msg_.speed;
                }
                if (has_data(LEFT_STEP, stamp_ns) && has_data(RIGHT_STEP, stamp_ns)){
                    speed_diff_ = left_speed_.x - right_speed_.x;
                    // Kalman this!
                    kalman_tracker_.add_speed_measurement(speed_diff_, static_cast<double>(stamp_ns));
                    update_gait_frequency(static_cast<double>(stamp_ns));
                }
                break;
        }
    }

    void PartialLoads::update_gait_frequency(double t_ns){
        // Measure the gait's angular frequency (w) directly from
        // step-alternation timing instead of leaving it to the Kalman
        // tracker's own weak, indirect observation of it (see
        // kalman/DiffSystemModel.hpp / FrequencyMeasurementModel.hpp): a
        // zero-crossing of speed_diff_ (left foot faster <-> right foot
        // faster) marks the moment the swing/stance roles swap, which
        // happens twice per gait cycle. The time between two consecutive
        // crossings is therefore half a gait period.
        bool crossed_zero = prev_speed_diff_valid_ &&
            ( (prev_speed_diff_for_zc_ < 0.0 && speed_diff_ > 0.0) ||
              (prev_speed_diff_for_zc_ > 0.0 && speed_diff_ < 0.0) );

        if (crossed_zero){
            double now_s = t_ns * 1e-9;
            if (has_last_crossing_){
                double half_period_s = now_s - last_crossing_time_s_;
                if (half_period_s > zero_crossing_refractory_s_){
                    double w_estimate = M_PI / half_period_s;   // period = 2*half_period; w = 2*pi/period
                    if (w_estimate >= w_estimate_min_ && w_estimate <= w_estimate_max_){
                        kalman_tracker_.add_frequency_measurement(w_estimate, t_ns);
                    }
                    last_crossing_time_s_ = now_s;
                }
                // else: too close to the last one to be a real swing/stance
                // swap (more likely noise wobbling around zero) -- ignored,
                // and last_crossing_time_s_ is deliberately left alone so
                // the next genuine crossing is still timed against the last
                // accepted one, not against this noisy one.
            } else {
                has_last_crossing_ = true;
                last_crossing_time_s_ = now_s;
            }
        }

        prev_speed_diff_for_zc_ = speed_diff_;
        prev_speed_diff_valid_ = true;
    }

    bool PartialLoads::has_data(int stream, int64_t stamp_ns){
        // an input that stopped publishing counts as "no data", not as the
        // last value it ever sent, however old that may be
        return last_stamp_ns_[stream] != kNoData &&
               stamp_ns - last_stamp_ns_[stream] <= data_timeout_ns_;
    }

    void PartialLoads::user_desc_lc(const walker_msgs::msg::UserDesc::SharedPtr msg)  {
        if (msg->weight <= 0){
            RCLCPP_ERROR(this->get_logger(), "Ignoring invalid user weight in user_desc (%d)", msg->weight);
            return;
        }
        weight_ = msg->weight;
        weight_received_ = true;
        release_inputs();
    }

    void PartialLoads::publish_loads(int64_t stamp_ns){
        double right_leg_load, left_leg_load;
        walker_msgs::msg::StepStamped msg;

        if (!new_data_available_){
            RCLCPP_DEBUG(this->get_logger(), "No new data received yet ...");
            return;
        }
        new_data_available_ = false;

        bool left_step_ok    = has_data(LEFT_STEP, stamp_ns);
        bool right_step_ok   = has_data(RIGHT_STEP, stamp_ns);
        bool left_handle_ok  = has_data(LEFT_HANDLE, stamp_ns);
        bool right_handle_ok = has_data(RIGHT_HANDLE, stamp_ns);

        if (!first_data_ready_ ){
            first_data_ready_ = ( left_step_ok & right_step_ok & left_handle_ok & right_handle_ok & weight_received_ );
        }

        // even once bootstrapped, don't compute/publish on stale
        // handle/step inputs: a stream that stopped publishing
        // shouldn't leave the last load frozen and reported as current
        // forever. weight_received_ is excluded here on purpose: unlike
        // the other four, user_desc is a one-shot latched value, not a
        // continuous stream, so it doesn't need to keep arriving.
        if ( first_data_ready_ && left_step_ok && right_step_ok && left_handle_ok && right_handle_ok ){
                // amount of weight on legs, clamped: sensor noise/out-of-range
                // handle readings must never produce a negative or over-100% load
                leg_load_ = weight_ - left_hand_msg_.force - right_hand_msg_.force;
                if (leg_load_ < 0.0){
                    leg_load_ = 0.0;
                } else if (leg_load_ > weight_){
                    leg_load_ = weight_;
                }

                double speed_diff = kalman_tracker_.get_speed_diff();

                // Assign weight to supporting leg(s). Instead of jumping
                // straight from 50/50 to 100/0 the instant |speed_diff|
                // crosses a threshold (which chatters when speed_diff is
                // noisy near it), ramp smoothly across a double-support
                // band. That band's width scales with the user's own
                // estimated swing-speed amplitude (v1 in the Kalman
                // model) instead of one fixed value for everyone: a slow
                // walker's speed_diff would otherwise never reach a
                // one-size-fits-all threshold, and a fast walker's would
                // blow past it immediately.
                //
                // v1 is only trustworthy once it has converged on
                // plausible, physically-bounded gait data; offline replay
                // against a real (if imperfect) labeled bag showed it can
                // still blow up to two orders of magnitude above any real
                // walking speed while walker_step_detector's own step
                // speed estimate is bad (tracked separately from this
                // package). speed_delta_max_ caps that: when v1 is
                // garbage, this collapses to a fixed, known-safe band
                // instead of propagating the garbage into the ratio.
                double amplitude = kalman_tracker_.get_speed_amplitude();
                double band = speed_delta_ratio_ * amplitude;
                if (band < speed_delta_){
                    band = speed_delta_;
                } else if (band > speed_delta_max_){
                    band = speed_delta_max_;
                }
                double ratio = speed_diff / band;
                if (ratio > 1.0){
                    ratio = 1.0;
                } else if (ratio < -1.0){
                    ratio = -1.0;
                }
                double right_fraction = 0.5 * (1.0 + ratio);

                right_leg_load = right_fraction * leg_load_;
                left_leg_load = (1.0 - right_fraction) * leg_load_;

                // Build msgs and publish
                // left
                msg = left_step_msg_;
                msg.load = left_leg_load;
                left_load_pub_->publish(msg);

                // right
                msg = right_step_msg_;
                msg.load = right_leg_load;
                right_load_pub_->publish(msg);
                RCLCPP_DEBUG(this->get_logger(), "Weight distribution on legs L(%3.3f) - R(%3.3f)",left_leg_load, right_leg_load);
        }else{
            RCLCPP_DEBUG(this->get_logger(), "Not all data received yet, or some of it went stale...");
        }
    }
