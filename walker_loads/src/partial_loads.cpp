#include "walker_loads/partial_loads.h"


PartialLoads::PartialLoads() : Node("partial_loads"){

        //Declare ROS parameters
        this->declare_parameter("handle_calibration_file");
        this->declare_parameter("left_loads_topic_name");
        this->declare_parameter("right_loads_topic_name");
        this->declare_parameter("left_handle_topic_name");
        this->declare_parameter("right_handle_topic_name");
        this->declare_parameter("left_steps_topic_name");
        this->declare_parameter("right_steps_topic_name");
        this->declare_parameter("user_desc_topic_name");
        this->declare_parameter("ms_period");
        this->declare_parameter("speed_delta");

        //Get ROS parameters
        this->get_parameter_or("scan_topic", scan_topic, std::string("/scan"));
        this->get_parameter_or("left_loads_topic_name", left_loads_topic_name_, std::string("/left_loads"));
        this->get_parameter_or("right_loads_topic_name", right_loads_topic_name_, std::string("/right_loads"));
        this->get_parameter_or("left_handle_topic_name", left_handle_topic_name_, std::string("/left_handle"));
        this->get_parameter_or("right_handle_topic_name", right_handle_topic_name_, std::string("/right_handle"));
        this->get_parameter_or("left_steps_topic_name", left_steps_topic_name_, std::string("/detected_step_left"));
        this->get_parameter_or("right_steps_topic_name", right_steps_topic_name_, std::string("/detected_step_right"));
        this->get_parameter_or("user_desc_topic_name", user_desc_topic_name_, std::string("/user_desc"));
        this->get_parameter_or("ms_period", ms_period_,  500);
        this->get_parameter_or("speed_delta", speed_delta_,  0.05);
        handle_calibration_file_ = ament_index_cpp::get_package_share_directory("walker_loads") + "/config/handle_calib.yaml";
        this->get_parameter_or("handle_calibration_file", handle_calibration_file_, handle_calibration_file_);

        //Load config files
        loadHandleCalibration();

        // Internal state data
        left_handle_msg.header.frame_id = "NONE";
        right_handle_msg.header.frame_id = "NONE";
        left_step_msg.position.header.frame_id = "NONE";
        right_step_msg.position.header.frame_id = "NONE";
        speed_diff = 0;
        weight = 100;
        right_handle_weight = 0;
        left_handle_weight  = 0;
        leg_load = 0;
        new_data_available = false;
        first_data_ready = false;    


/*
        // Load kalman tracker
        kalman_tracker.init(this, d0, a0, f0, p0 );

        // Set debug output
        if (is_debug){
            kalman_tracker.enable_log();

            auto ret = rcutils_logging_set_logger_level( this->get_logger().get_name(), RCUTILS_LOG_SEVERITY_DEBUG);
            if (ret != RCUTILS_RET_OK) {
                RCLCPP_ERROR(this->get_logger(), "Error setting severity: %s", rcutils_get_error_string().str);
                rcutils_reset_error();
            }

            RCLCPP_INFO(this->get_logger(), "kalman model initial d: %.2f", d0);
        } else {
            RCLCPP_INFO(this->get_logger(), "Step detector loading. Set plot vars to true for debug.");
        }
*/
        // ROS Comms
        auto default_qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());

        // Publishers       
        left_load_pub_  = this->create_publisher<walker_msgs::msg::StepStamped>(left_loads_topic_name_, 20);
        right_load_pub_ = this->create_publisher<walker_msgs::msg::StepStamped>( right_loads_topic_name_, 20);

        // Subscribers
        left_handle_sub_ = this->create_subscription<walker_msgs::msg::ForceStamped>( left_handle_topic_name_, default_qos, std::bind(&PartialLoads::handle_lc, this, std::placeholders::_1));

//        right_handle_sub_ = this->create_subscription<walker_msgs::msg::ForceStamped>( right_handle_topic_name_, default_qos, std::bind(&PartialLoads::handle_lc, this, std::placeholders::_1));
//        left_steps_sub_ = this->create_subscription<walker_msgs::msg::StepStamped>( left_steps_topic_name_, default_qos, std::bind(&PartialLoads::l_steps_lc, this, std::placeholders::_1));
//        right_steps_sub_ = this->create_subscription<walker_msgs::msg::StepStamped>( right_steps_topic_name_, default_qos, std::bind(&PartialLoads::r_steps_lc, this, std::placeholders::_1));
//        user_desc_sub_ = this->create_subscription<std_msgs::msg::String>( user_desc_topic_name_, default_qos, std::bind(&PartialLoads::user_desc_lc, this, std::placeholders::_1));

        // timers
        timer_ = create_wall_timer( std::chrono::milliseconds(ms_period_), std::bind(&PartialLoads::timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "load detector started");
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
				
                if (param_name.find("left") != std::string::npos){
					left_handle_points_  = param.get_value<std::vector<double>>();
				}

                if (param_name.find("right") != std::string::npos){
					right_handle_points_  = param.get_value<std::vector<double>>();
				}

                if (param_name.find("weight") != std::string::npos){
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
        
        /*
		for (double di = 0.0; di < 1000; di+=50) {
			double wl = fl_.interp(di);
			RCLCPP_INFO(this->get_logger(), "Left: Measure (%2.2f) - Weight (%2.2f) Kg", di, wl);
		}
		for (double di = 0.0; di < 1000; di+=50) {
			double wr = fr_.interp(di);
			RCLCPP_INFO(this->get_logger(), "Right: Measure (%2.2f) - Weight (%2.2f) Kg", di, wr);
		}	
        */
    }

    void PartialLoads::handle_lc(const walker_msgs::msg::ForceStamped::SharedPtr msg)  {

    }

    void PartialLoads::l_steps_lc(const walker_msgs::msg::StepStamped::SharedPtr msg)  {

    }

    void PartialLoads::r_steps_lc(const walker_msgs::msg::StepStamped::SharedPtr msg)  {

    }

    void PartialLoads::user_desc_lc(const std_msgs::msg::String::SharedPtr msg)  {

    }

    void PartialLoads::timer_callback(){

    }

