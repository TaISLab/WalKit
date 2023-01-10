#include "walker_step_detector/detect_steps_s.h"



KMDetectSteps::KMDetectSteps() : Node("detect_steps"){
        //Get ROS parameters
        this->declare_parameter<std::string>("segments_topic",            "/segments");
        this->declare_parameter<std::string>("detected_steps_topic_name", "/detected_step");
        this->declare_parameter<std::string>("detected_steps_frame",      "/base_link");
        this->declare_parameter<double>("kalman_model_d0",                0.001);
        this->declare_parameter<double>("kalman_model_a0",                0.001);
        this->declare_parameter<double>("kalman_model_f0",                0.001);
        this->declare_parameter<double>("kalman_model_p0",                0.001);
        this->declare_parameter<bool>("plot_leg_kalman",                  false);
        this->declare_parameter<bool>("plot_leg_clusters",                false);
        this->declare_parameter<bool>("use_segment_header_stamp_for_tfs", false);


        this->get_parameter("segments_topic",                    segments_topic_);
        this->get_parameter("detected_steps_topic_name",         detected_steps_topic_name_);
        this->get_parameter("detected_steps_frame",              detected_steps_frame_);
        this->get_parameter("kalman_model_d0_",                  kalman_model_d0_);
        this->get_parameter("kalman_model_a0_",                  kalman_model_a0_);
        this->get_parameter("kalman_model_f0_",                  kalman_model_f0_);
        this->get_parameter("kalman_model_p0_",                  kalman_model_p0_);
        this->get_parameter("plot_leg_kalman",                   plot_leg_kalman_);
        this->get_parameter("plot_leg_clusters",                 plot_leg_clusters_);
        this->get_parameter("use_segment_header_stamp_for_tfs",  use_segment_header_stamp_for_tfs_);


        // Load kalman tracker
        kalman_tracker.init(this, kalman_model_d0_, kalman_model_a0_, kalman_model_f0_, kalman_model_p0_ );

        // Verbose init
        is_debug = (plot_leg_clusters_|| plot_leg_kalman_);
        if (is_debug){
            kalman_tracker.enable_log();

            auto ret = rcutils_logging_set_logger_level( this->get_logger().get_name(), RCUTILS_LOG_SEVERITY_DEBUG);
            if (ret != RCUTILS_RET_OK) {
                RCLCPP_ERROR(this->get_logger(), "Error setting severity: [%s]", rcutils_get_error_string().str);
                rcutils_reset_error();
            }

            RCLCPP_INFO(this->get_logger(), "kalman model initial d: [%.2f]", kalman_model_d0_);
            RCLCPP_INFO(this->get_logger(), "kalman model initial a: [%.2f]", kalman_model_a0_);
            RCLCPP_INFO(this->get_logger(), "kalman model initial f: [%.2f]", kalman_model_f0_);
            RCLCPP_INFO(this->get_logger(), "kalman model initial p: [%.2f]", kalman_model_p0_);
            //Print the ROS parameters
            RCLCPP_INFO(this->get_logger(), "segments_topic: [%s]", segments_topic_.c_str());
            RCLCPP_INFO(this->get_logger(), "detected_steps_topic_name: [%s]", detected_steps_topic_name_.c_str());
            RCLCPP_INFO(this->get_logger(), "detected_steps_frame: [%s]", detected_steps_frame_.c_str());
        } else {
            RCLCPP_INFO(this->get_logger(), "Step detector loading. Set plot vars to true for debug.");
        }

        //ROS STUF
        auto default_qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());

        // publishers
        left_detected_step_pub_ = this->create_publisher<walker_msgs::msg::StepStamped>(detected_steps_topic_name_ + "_left", 20);
        right_detected_step_pub_ = this->create_publisher<walker_msgs::msg::StepStamped>(detected_steps_topic_name_ + "_right", 20);
        if (is_debug){
            markers_array_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("clusters", 20);
        }

        // transform buffer
        buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tfl_ = std::make_shared<tf2_ros::TransformListener>(*buffer_);
        auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
                                                            this->get_node_base_interface(),
                                                            this->get_node_timers_interface());
        buffer_->setCreateTimerInterface(timer_interface);

        //publishers last
        this->segments_sub_ = this->create_subscription<slg_msgs::msg::SegmentArray>(segments_topic_, default_qos, std::bind(&KMDetectSteps::segmentsCallback, this, std::placeholders::_1));

    }


std::list<walker_msgs::msg::StepStamped> KMDetectSteps::getCentroids(std::string  fixed_frame_id, slg_msgs::msg::SegmentArray::SharedPtr segments_msg, std::shared_ptr<tf2_ros::Buffer> tf_buff){
        std::list<walker_msgs::msg::StepStamped> centroids;

        for (const auto &segment_msg : segments_msg->segments){
            slg::Segment2D current_segment = segment_msg;
            geometry_msgs::msg::PointStamped position;

            position.header = segments_msg->header;
            position.point.x = current_segment.centroid().x;
            position.point.y = current_segment.centroid().y;

            // transform into requested frame
            try {
                tf_buff->transform(position, position, fixed_frame_id);
            } catch (tf2::TransformException &e){
                RCLCPP_ERROR (this->get_logger(), "Cant transform [%s]", e.what());
            }

            walker_msgs::msg::StepStamped new_step;
            new_step.position = position;

            new_step.confidence = 1.0;

            centroids.push_back(new_step);
            
        }
        return centroids;
    }

void KMDetectSteps::segmentsCallback(const slg_msgs::msg::SegmentArray::SharedPtr seg_array)  {
        walker_msgs::msg::StepStamped left_detected_step;
        walker_msgs::msg::StepStamped right_detected_step;

        // some debug info about the received segments
        if (is_debug){
            //RCLCPP_DEBUG(this->get_logger(), "%d clusters with distances bigger than %3.3f", processor.size(), cluster_dist_euclid_); 
        }


        // Find out which timestamp should be used for tfs
        bool transform_available;
        rclcpp::Time tf_time;

        // Use time from segment header
        if (use_segment_header_stamp_for_tfs_){
            tf_time = seg_array->header.stamp;
            try {
                buffer_->lookupTransform(detected_steps_frame_, seg_array->header.frame_id, tf_time, rclcpp::Duration::from_seconds(1.0));
                transform_available = buffer_->canTransform(detected_steps_frame_, seg_array->header.frame_id, tf_time);              
            } catch(tf2::TransformException &e) {
                RCLCPP_WARN(this->get_logger(), "No tf available");
                transform_available = false;
                
            }
        } else {
            // Otherwise just use the latest tf available
            transform_available = buffer_->canTransform(detected_steps_frame_, seg_array->header.frame_id, tf_time);
        }

        if(!transform_available) {
            RCLCPP_WARN(this->get_logger(), "No TF available: step position will be extrapolated");
        } else {     

            // clear rviz 
            if (is_debug){
                delete_markers();
            }

            std::list<walker_msgs::msg::StepStamped> points = getCentroids(detected_steps_frame_, seg_array, buffer_);
            kalman_tracker.add_detections(points);
        }

        // get steps from Kalman set
        walker_msgs::msg::StepStamped step_r;
        walker_msgs::msg::StepStamped step_l;
        double t = (this->now()).nanoseconds();

        kalman_tracker.get_steps(&step_r, &step_l, t);

        // publish lets
        if (kalman_tracker.is_init){
            right_detected_step_pub_->publish(step_r);
            left_detected_step_pub_->publish(step_l);
        }

        if (is_debug){
            publish_leg(step_r, 0);
            publish_leg(step_l, 1);
            RCLCPP_DEBUG(this->get_logger(), ".......\n\n"); 
        }
    }

void KMDetectSteps::delete_markers(){        
        visualization_msgs::msg::Marker marker_cluster;
        visualization_msgs::msg::MarkerArray marker_array;

        marker_cluster.action = visualization_msgs::msg::Marker::DELETEALL;
        marker_array.markers.push_back(marker_cluster);

        markers_array_pub_->publish(marker_array);
}

void KMDetectSteps::publish_leg(walker_msgs::msg::StepStamped step, int sid){
        visualization_msgs::msg::MarkerArray marker_array;
        visualization_msgs::msg::Marker mark_i = get_marker(&step, sid);

        mark_i.color.r = (sid==1 ? 1.0 : 0.0); 
        mark_i.color.g = 0; 
        mark_i.color.b = (sid==0 ? 1.0 : 0.0); 

        marker_array.markers.push_back(mark_i);
        markers_array_pub_->publish(marker_array);
    }

visualization_msgs::msg::Marker KMDetectSteps::get_marker(const walker_msgs::msg::StepStamped* step, int id ){       
        std::string ns = "steps";
        
        RGB rgb0 = pick_one(id); 

        double r = rgb0.r; 
        double g = rgb0.g; 
        double b = rgb0.b;

        double size = 0.05;

        visualization_msgs::msg::Marker marker;

        marker.header = step->position.header;
        marker.lifetime = rclcpp::Duration(std::chrono::nanoseconds(0));

        marker.ns = ns;
        marker.id = id;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.type = visualization_msgs::msg::Marker::CUBE;

        marker.pose.position.x = step->position.point.x;
        marker.pose.position.y = step->position.point.y;
        marker.pose.position.z = step->position.point.z;        
        marker.pose.orientation.w = 1;

        marker.scale.x = size;
        marker.scale.y = size;
        marker.scale.z = size;        

        marker.color.r = r;
        marker.color.g = g;
        marker.color.b = b;
        marker.color.a = 1;

        return marker;
    }

int main(int argc, char **argv)
{

    rclcpp::init(argc, argv);
    auto node = std::make_shared<KMDetectSteps>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
