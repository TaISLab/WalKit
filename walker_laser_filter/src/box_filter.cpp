#include "walker_laser_filter/box_filter.hpp"



BoxFilter::BoxFilter(rclcpp::NodeOptions options=rclcpp::NodeOptions()) : Node("box_filter",options.allow_undeclared_parameters(true).automatically_declare_parameters_from_overrides(true)){

        params_interface_ = get_node_parameters_interface();

        /*
        
        std::map<std::string, rclcpp::Parameter> param_map;        
        bool result = get_parameters("", param_map );
        RCLCPP_INFO(get_logger(), "[%s]: Available params:",      get_name());        
        for (auto & param_i : param_map) {
            RCLCPP_INFO(get_logger(), "[%s]: '%s'",      get_name(), param_i.second.get_name().c_str());
        }
        */

        //ROS parameters
        param_prefix_ = "filter."; // TODO: find a smarter way to get this
        initParam(param_prefix_ + "in_topic",      "/scan",          in_scan_topic_name_);
        initParam(param_prefix_ + "out_topic",     "/scan_filtered", out_scan_topic_name_);
        initParam(param_prefix_ + "remove_out",    true,             remove_out_);
        initParam(param_prefix_ + "params.max_x",   0.03,            max_x_);
        initParam(param_prefix_ + "params.max_y",   0.3,             max_y_);
        initParam(param_prefix_ + "params.min_x",  -1.2,             min_x_);
        initParam(param_prefix_ + "params.min_y",  -0.3,             min_y_);

        // show
        RCLCPP_INFO(get_logger(), "[%s]: Current config:",      get_name());        
        RCLCPP_INFO(get_logger(), "[%s]: in_topic: [%s]",       get_name(), in_scan_topic_name_.c_str());
        RCLCPP_INFO(get_logger(), "[%s]: out_topic: [%s]",      get_name(), out_scan_topic_name_.c_str());        
        RCLCPP_INFO(get_logger(), "[%s]: max_x: [%3.3f]",       get_name(), max_x_);
        RCLCPP_INFO(get_logger(), "[%s]: max_y: [%3.3f]",       get_name(), max_y_);
        RCLCPP_INFO(get_logger(), "[%s]: min_x: [%3.3f]",       get_name(), min_x_);
        RCLCPP_INFO(get_logger(), "[%s]: min_y: [%3.3f]",       get_name(), min_y_);  
        RCLCPP_INFO(get_logger(), "[%s]: remove_out: [%s]",     get_name(), (remove_out_ ? "true": "false")  );

        // Internal state data

        // ROS Comms
        auto default_qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());

        // Publishers       
        this->out_scan_pub_  = this->create_publisher<sensor_msgs::msg::LaserScan>(out_scan_topic_name_, 20);

        // Subscribers
        this->in_scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(in_scan_topic_name_, default_qos, std::bind(&BoxFilter::scanCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Box Filter started");
    }

bool BoxFilter::isValid(double x, double y) {
    bool ans;
    if (remove_out_){
        ans = (x < max_x_) && (x > min_x_) &&
              (y < max_y_) && (y > min_y_);
    } else {
        ans = (x > max_x_) && (x < min_x_) &&
              (y > max_y_) && (y < min_y_);
    }

    return ans;
    }

void BoxFilter::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
      
    sensor_msgs::msg::LaserScan scan_out;
    scan_out = *scan;

    for (unsigned long int i = 0; i < scan->ranges.size(); i++) {
        double px, py,range;
        bool in_range;
        range = scan->ranges[i];
        in_range  = (range > scan->range_min && range < scan->range_max);
        
        if (in_range){
            px = cos( scan->angle_min + i * scan->angle_increment ) * range;
            py = sin( scan->angle_min + i * scan->angle_increment ) * range;
            if (!isValid(px,py)){
                scan_out.ranges[i] = std::numeric_limits<float>::quiet_NaN();;
            }
        }        
    }
    this->out_scan_pub_->publish(scan_out);
}


 bool BoxFilter::initParam(const std::string & name, std::string value_default, std::string & value_out){
    return initParamImpl(
      name, rcl_interfaces::msg::ParameterType::PARAMETER_STRING, value_default, value_out);
  }
 
 bool BoxFilter::initParam(const std::string & name, bool value_default, bool & value_out){
    return initParamImpl(
      name, rcl_interfaces::msg::ParameterType::PARAMETER_BOOL, value_default, value_out);
  }

 bool BoxFilter::initParam(const std::string & name, double value_default, double & value_out){
    return initParamImpl(
      name, rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE, value_default, value_out);
  }

  template<typename PT>
  bool BoxFilter::initParamImpl(const std::string & name, const uint8_t type, PT default_value, PT & value_out){
    //std::string param_name = param_prefix_ + name;
    std::string param_name = name;

    if (!params_interface_->has_parameter(param_name)) {
      RCLCPP_WARN(get_logger(), "[%s]: param [%s] not found. Using default value", get_name(), param_name.c_str());  
      // Declare parameter
      rclcpp::ParameterValue default_parameter_value(default_value);
      rcl_interfaces::msg::ParameterDescriptor desc;
      desc.name = name;
      desc.type = type;
      desc.read_only = true;

      if (name.empty()) {
        throw std::runtime_error("Parameter must have a name");
      }

      params_interface_->declare_parameter(param_name, default_parameter_value, desc);
    }

    value_out = params_interface_->get_parameter(param_name).get_parameter_value().get<PT>();
    // TODO(sloretz) seems to be no way to tell if parameter was initialized or not
    return true;
  }

int main(int argc, char **argv){ 
 	rclcpp::init(argc, argv);
	auto node = std::make_shared<BoxFilter>();	
    RCLCPP_INFO(node->get_logger(), "Starting BoxFilter node. ");

    rclcpp::WallRate loop_rate(200);
    while (rclcpp::ok()) {
        rclcpp::spin_some(node);
        loop_rate.sleep();
    }

    RCLCPP_INFO(node->get_logger(), "Shutting down BoxFilter node. ");
	rclcpp::shutdown();

    return 0;
}
