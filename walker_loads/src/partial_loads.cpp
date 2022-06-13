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
        this->declare_parameter("debug_output");

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
        this->get_parameter_or("debug_output", debug_output_, false);


        //Load config files
        loadHandleCalibration();

        // Internal state data
        left_handle_msg_.header.frame_id = "NONE";
        right_handle_msg_.header.frame_id = "NONE";
        left_step_msg_.position.header.frame_id = "NONE";
        right_step_msg_.position.header.frame_id = "NONE";
        speed_diff_ = 0;
        weight_ = 100;
        right_handle_weight_ = 0;
        left_handle_weight_  = 0;
        leg_load_ = 0;
        new_data_available_ = false;
        first_data_ready_ = false;    



        // Load kalman tracker
        double v0 = 0.01;   // meters/s
        double v1 = 1.0;    // meters/s
        double f0 = 4.0;    // Kg.
        double f1 = 15.0;   // Kg.
        double w = 0.1;    // rad/s
        double d = 0.50;   // rad
        double vp = 0.20;   // rad

        kalman_tracker_.init(this,"diff_tracker", v0, v1, f0, f1, w,  d, vp);

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
        auto default_qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());

        // Publishers       
        left_load_pub_  = this->create_publisher<walker_msgs::msg::StepStamped>(left_loads_topic_name_, 20);
        right_load_pub_ = this->create_publisher<walker_msgs::msg::StepStamped>( right_loads_topic_name_, 20);

        // Subscribers
        left_handle_sub_ = this->create_subscription<walker_msgs::msg::ForceStamped>( left_handle_topic_name_, default_qos, std::bind(&PartialLoads::handle_lc, this, std::placeholders::_1));
        right_handle_sub_ = this->create_subscription<walker_msgs::msg::ForceStamped>( right_handle_topic_name_, default_qos, std::bind(&PartialLoads::handle_lc, this, std::placeholders::_1));
        left_steps_sub_ = this->create_subscription<walker_msgs::msg::StepStamped>( left_steps_topic_name_, default_qos, std::bind(&PartialLoads::l_steps_lc, this, std::placeholders::_1));
        right_steps_sub_ = this->create_subscription<walker_msgs::msg::StepStamped>( right_steps_topic_name_, default_qos, std::bind(&PartialLoads::r_steps_lc, this, std::placeholders::_1));
        user_desc_sub_ = this->create_subscription<std_msgs::msg::String>( user_desc_topic_name_, default_qos, std::bind(&PartialLoads::user_desc_lc, this, std::placeholders::_1));

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
        
    }

    void PartialLoads::handle_lc(const walker_msgs::msg::ForceStamped::SharedPtr msg)  {
        std::string frame_id = msg->header.frame_id;

        if (frame_id.find("right") != std::string::npos){
            right_handle_msg_ = *msg;
            right_handle_weight_ = fr_.interp(msg->force);
            new_data_available_ = true;
        } else if (frame_id.find("left") != std::string::npos){
            left_handle_msg_ = *msg;
            left_handle_weight_ = fl_.interp(msg->force);
            new_data_available_ = true;
        } else{
            RCLCPP_ERROR(this->get_logger(), "Don't know about which handle are you talking [%s]", frame_id.c_str());
            return;
        }

        if ( has_data(left_handle_msg_.header) &  has_data(right_handle_msg_.header) ){ 
            double force_diff = left_handle_weight_ - right_handle_weight_;
            // Kalman this!
            double t = (this->now()).nanoseconds();
            kalman_tracker_.add_force_measurement(force_diff, t);
        }        

    }

    void PartialLoads::l_steps_lc(const walker_msgs::msg::StepStamped::SharedPtr msg)  {
        steps_lc(msg, 0);
    }

    void PartialLoads::r_steps_lc(const walker_msgs::msg::StepStamped::SharedPtr msg)  {
        steps_lc(msg, 0);
    }

    void PartialLoads::steps_lc(const walker_msgs::msg::StepStamped::SharedPtr msg, int id){
        if (id==1){
            right_step_msg_ = *msg;
            right_speed_ = right_step_msg_.speed;
            new_data_available_ = true;
        } else if (id == 0) {
            left_step_msg_ = *msg;
            left_speed_ = left_step_msg_.speed;
            new_data_available_ = true;
        } else {
            RCLCPP_ERROR(this->get_logger(), "Don't know about which step are you talking [%d]", id);
            return;
        }

        if ( has_data(left_step_msg_.position.header) &  has_data(right_step_msg_.position.header) ){ 
            speed_diff_ = left_speed_.x - right_speed_.x ;
            // Kalman this!
            double t = (this->now()).nanoseconds();
            kalman_tracker_.add_speed_measurement(speed_diff_,t);
        }
    }

    bool PartialLoads::has_data( std_msgs::msg::Header header){
        return (header.frame_id.find("NONE") == std::string::npos);
    }

    void PartialLoads::user_desc_lc(const std_msgs::msg::String::SharedPtr msg)  {
        std::string data = msg->data;
        auto tokens = rcpputils::split(data, ':');
       
        if (tokens.size()>2){
            int new_weight = atoi(tokens[2].c_str());
            if (new_weight != weight_){
                weight_ = new_weight;
                new_data_available_ = true;
            }
        }

    }

    void PartialLoads::timer_callback(){
        double right_leg_load, left_leg_load;
        walker_msgs::msg::StepStamped msg;
        if (new_data_available_){
            new_data_available_ = false;

            if (!first_data_ready_ ){
                first_data_ready_ = ( has_data(left_step_msg_.position.header)  &  
                                      has_data(right_step_msg_.position.header) &
                                      has_data(left_handle_msg_.header) &
                                      has_data(right_handle_msg_.header) );
            }

            if ( first_data_ready_ ){
                    // amount of weight on legs
                    leg_load_ = weight_ - left_handle_weight_ - right_handle_weight_;
                    
                    speed_diff_ = kalman_tracker_.get_speed_diff();

                    // assign weight to supporting leg...
                    if (speed_diff_>speed_delta_){
                        right_leg_load = leg_load_;
                        left_leg_load = 0.0;
                    } else if (speed_diff_<- speed_delta_){
                        right_leg_load = 0.0;
                        left_leg_load = leg_load_;
                    } else{
                        // Both leg supporting: We can't be sure about how much on each one!
                        right_leg_load = 0.5 * leg_load_;
                        left_leg_load =  0.5 * leg_load_;
                    }

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
                RCLCPP_DEBUG(this->get_logger(), "Not all data received yet ...");
            }
        } else {
            RCLCPP_DEBUG(this->get_logger(), "No new data received yet ...");
        }
    }

