#include "walker_step_detector/detect_steps_s.h"

DetectStepsS::DetectStepsS() : Node("detect_steps"){
    //Get ROS parameters
    this->declare_parameter<std::string>("segments_topic",            "/segments");
    this->declare_parameter<std::string>("detected_steps_topic_name", "/detected_step");
    this->declare_parameter<double>("kalman_model_d0",                0.001);
    this->declare_parameter<double>("kalman_model_a0",                0.001);
    this->declare_parameter<double>("kalman_model_f0",                0.001);
    this->declare_parameter<double>("kalman_model_p0",                0.001);
    this->declare_parameter<bool>("kalman_enabled",                   false);
    this->declare_parameter<bool>("is_debug",                         false);


    this->get_parameter("segments_topic",                    segments_topic_);
    this->get_parameter("detected_steps_topic_name",         detected_steps_topic_name_); 
    this->get_parameter("kalman_model_d0_",                  kalman_model_d0_);
    this->get_parameter("kalman_model_a0_",                  kalman_model_a0_);
    this->get_parameter("kalman_model_f0_",                  kalman_model_f0_);
    this->get_parameter("kalman_model_p0_",                  kalman_model_p0_);
    this->get_parameter("kalman_enabled",                    kalman_enabled_);
    this->get_parameter("is_debug",                          is_debug);

    // Load kalman tracker
    kalman_tracker.init(this, kalman_model_d0_, kalman_model_a0_, kalman_model_f0_, kalman_model_p0_ );
    kalman_tracker.set_status(kalman_enabled_);

    // Verbose init
    if (is_debug){
        kalman_tracker.enable_log();

        auto ret = rcutils_logging_set_logger_level( this->get_logger().get_name(), RCUTILS_LOG_SEVERITY_DEBUG);
        if (ret != RCUTILS_RET_OK) {
            RCLCPP_ERROR(this->get_logger(), "Error setting severity: [%s]", rcutils_get_error_string().str);
            rcutils_reset_error();
        }

        //Printing ROS parameters
        RCLCPP_INFO(this->get_logger(), "segments_topic: [%s]", segments_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "detected_steps_topic_name: [%s]", detected_steps_topic_name_.c_str());
        RCLCPP_INFO(this->get_logger(), "kalman model: ");
        RCLCPP_INFO(this->get_logger(), "       - active: [%s]: ", (kalman_enabled_) ? ("YES") : ("NO"));
        RCLCPP_INFO(this->get_logger(), "       - d: %.2f", kalman_model_d0_);
        RCLCPP_INFO(this->get_logger(), "       - a: %.2f", kalman_model_a0_);
        RCLCPP_INFO(this->get_logger(), "       - f: %.2f", kalman_model_f0_);
        RCLCPP_INFO(this->get_logger(), "       - p: %.2f", kalman_model_p0_);
    } else {
        RCLCPP_INFO(this->get_logger(), "Step detector loading. Set is_debug to true for debug.");
    }

    //ROS STUFF
    auto default_qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());

    // publishers
    left_detected_step_pub_ = this->create_publisher<walker_msgs::msg::StepStamped>(detected_steps_topic_name_ + "_left", 20);
    right_detected_step_pub_ = this->create_publisher<walker_msgs::msg::StepStamped>(detected_steps_topic_name_ + "_right", 20);

    //subscribers last
    this->segments_sub_ = this->create_subscription<slg_msgs::msg::SegmentArray>(segments_topic_, default_qos, std::bind(&DetectStepsS::segmentsCallback, this, std::placeholders::_1));

}

void DetectStepsS::segmentsCallback(const slg_msgs::msg::SegmentArray::SharedPtr seg_array)  {
    walker_msgs::msg::StepStamped left_detected_step;
    walker_msgs::msg::StepStamped right_detected_step;

    RCLCPP_WARN(this->get_logger(), "Segment data received at: [%3.3f]", seg_array->header.stamp.sec + (seg_array->header.stamp.nanosec*1e-9));    

    std::list<walker_msgs::msg::StepStamped> points = getCentroids(seg_array);
    kalman_tracker.add_detections(points);

    // get steps from Kalman set
    RCLCPP_DEBUG(this->get_logger(), "Getting filtered positions from kalman");
    walker_msgs::msg::StepStamped step_r;
    walker_msgs::msg::StepStamped step_l;
    double t = (this->now()).nanoseconds();
    kalman_tracker.get_steps(&step_r, &step_l, t);

    // publish lets
    if (kalman_tracker.is_init){
        if (step_r.position.header.frame_id.compare("invalid") != 0){
            right_detected_step_pub_->publish(step_r);
            RCLCPP_WARN(this->get_logger(), "Right feet at: [%3.3f, %3.3f, %3.3f] [%s]", step_r.position.point.x, step_r.position.point.y, step_r.position.point.z, step_r.position.header.frame_id.c_str());    
        }
        if (step_l.position.header.frame_id.compare("invalid") != 0){    
            left_detected_step_pub_->publish(step_l);
            RCLCPP_WARN(this->get_logger(), "Left feet at: [%3.3f, %3.3f, %3.3f] [%s]", step_l.position.point.x, step_l.position.point.y, step_l.position.point.z, step_l.position.header.frame_id.c_str());    
        }
    }
    RCLCPP_WARN(this->get_logger(), "\n\n"); 
}

std::list<walker_msgs::msg::StepStamped> DetectStepsS::getCentroids(slg_msgs::msg::SegmentArray::SharedPtr segments_msg){
    std::list<walker_msgs::msg::StepStamped> centroids;

    for (const auto &segment_msg : segments_msg->segments){
        slg::Segment2D current_segment = segment_msg;
        geometry_msgs::msg::PointStamped position;

        position.header = segments_msg->header;
        position.point.x = current_segment.centroid().x;
        position.point.y = current_segment.centroid().y;

        walker_msgs::msg::StepStamped new_step;
        new_step.position = position;

        new_step.confidence = 1.0;

        centroids.push_back(new_step);
        
    }
    return centroids;
    }

int main(int argc, char **argv)
{

    rclcpp::init(argc, argv);
    auto node = std::make_shared<DetectStepsS>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
