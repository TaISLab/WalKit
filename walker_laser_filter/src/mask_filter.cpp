
#include "walker_laser_filter/mask_filter.hpp"



MaskFilter::MaskFilter(rclcpp::NodeOptions options=rclcpp::NodeOptions()) : Node("mask_filter",options.allow_undeclared_parameters(true).automatically_declare_parameters_from_overrides(true)){
        
        // std::map<std::string, rclcpp::Parameter> param_map;        
        // bool result = get_parameters("", param_map );
        // if (result) {
        //     RCLCPP_INFO(get_logger(), "[%s]: Available params:",      get_name());                
        //     for (auto & param_i : param_map) {
        //         RCLCPP_INFO(get_logger(), "[%s]: '%s'",      get_name(), param_i.second.get_name().c_str());
        //     }
        // }

    params_interface_ = get_node_parameters_interface();
    //Get ROS parameters
    param_prefix_ = "filter."; // TODO: find a smarter way to get this
    initParam(param_prefix_ + "in_topic",             "/scan",          in_scan_topic_name_);
    initParam(param_prefix_ + "out_topic",            "/scan_filtered", out_scan_topic_name_);
    initParam(param_prefix_ + "remove_listed", true,                    remove_listed_);

    // frame_id is implicit as the parameter name listint the removed points
    std::map<std::string, rclcpp::Parameter> param_map;        
    bool result = get_parameters("", param_map );
    std::string key_masks = param_prefix_ + "params.masks";

    if (result) {
        for (auto & param_i : param_map) {
            std::string param_name = param_i.second.get_name();
            if (param_name.find(key_masks)!=std::string::npos){
                // keep last token
                std::istringstream tokenStream(param_name);
                while (std::getline(tokenStream, frame_id_, '.')){};

                std::vector<double> values;
                initParam(param_name, values);
                masks_[frame_id_].clear();
                for (size_t i = 0; i < values.size(); ++i) {
                    size_t id = static_cast<int>(values[i]);
                    masks_[frame_id_].push_back(id);
                }
            }            
        }
    }

    if (masks_.empty()) {
      RCLCPP_FATAL(get_logger(), "[%s]: masks is not defined in parameters.",get_name());
    }

    // show
    RCLCPP_INFO(get_logger(), "[%s]: Current config: ", get_name());        
    RCLCPP_INFO(get_logger(), "[%s]: in_topic: [%s]", get_name(), in_scan_topic_name_.c_str());
    RCLCPP_INFO(get_logger(), "[%s]: out_topic: [%s]", get_name(), out_scan_topic_name_.c_str());
    RCLCPP_INFO(get_logger(), "[%s]: remove_listed: [%s]", get_name(), (remove_listed_ ? "true": "false")  );
    RCLCPP_INFO(get_logger(), "[%s]: %d directions will be masked in frame id [%s].", get_name(), (int)masks_[frame_id_].size(),frame_id_.c_str());
    
    // ROS Comms
    auto default_qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());

    // Publishers       
    this->out_scan_pub_  = this->create_publisher<sensor_msgs::msg::LaserScan>(out_scan_topic_name_, 20);

    // Subscribers
    this->in_scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(in_scan_topic_name_, default_qos, std::bind(&MaskFilter::scanCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "[%s] Mask Filter started", get_name());
}

void MaskFilter::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
      
    sensor_msgs::msg::LaserScan scan_out;
    scan_out = *scan;

    if (masks_.find(scan_out.header.frame_id) == masks_.end()){
      RCLCPP_WARN(get_logger(), "[%s]: frame_id %s is not registered.", get_name(), scan_out.header.frame_id.c_str());
    } else{
        const std::vector<size_t> &mask = masks_[scan_out.header.frame_id];
        const size_t len = scan_out.ranges.size();
        for (std::vector<size_t>::const_iterator it = mask.begin(); it != mask.end(); ++it){
            if (*it > len) continue;  // MFC: this shouldn't be possible on a correct config
            scan_out.ranges[*it] = std::numeric_limits<float>::quiet_NaN();
        }   
    }
    this->out_scan_pub_->publish(scan_out);
}


// based on getParam from
// https://github.com/ros/filters/blob/ros2/include/filters/filter_base.hpp
 bool MaskFilter::initParam(const std::string & name, std::vector<double> & value){
    return initParamImpl(
      name, rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE_ARRAY, {}, value);
  }

 bool MaskFilter::initParam(const std::string & name, std::string value_default, std::string & value_out){
    return initParamImpl(
      name, rcl_interfaces::msg::ParameterType::PARAMETER_STRING, value_default, value_out);
  }

 bool MaskFilter::initParam(const std::string & name, bool value_default, bool & value_out){
    return initParamImpl(
      name, rcl_interfaces::msg::ParameterType::PARAMETER_BOOL, value_default, value_out);
  }

  template<typename PT>
  bool MaskFilter::initParamImpl(const std::string & name, const uint8_t type, PT default_value, PT & value_out){
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

    try { 
        value_out = params_interface_->get_parameter(param_name).get_parameter_value().get<PT>();
    } catch (const rclcpp::ParameterTypeException& e) { 
        RCLCPP_FATAL(get_logger(), "[%s]: Error while init param [%s]: [%s]", get_name(), param_name.c_str(), e.what());  
    }
    
    // TODO(sloretz) seems to be no way to tell if parameter was initialized or not
    return true;
  }

int main(int argc, char **argv){ 
 	rclcpp::init(argc, argv);
	auto node = std::make_shared<MaskFilter>();	
    RCLCPP_INFO(node->get_logger(), "Starting MaskFilter node. ");

    rclcpp::WallRate loop_rate(200);
    while (rclcpp::ok()) {
        rclcpp::spin_some(node);
        loop_rate.sleep();
    }

    RCLCPP_INFO(node->get_logger(), "Shutting down MaskFilter node. ");
	rclcpp::shutdown();

    return 0;
}
