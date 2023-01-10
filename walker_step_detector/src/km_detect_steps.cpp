#include "walker_step_detector/km_detect_steps.h"

#include "walker_step_detector/detect_steps_s.h"



KMDetectSteps::KMDetectSteps() : Node("detect_steps"){
    //Get ROS parameters
    this->declare_parameter<std::string>("scan_topic",                "/scan");
    this->declare_parameter<std::string>("detected_steps_topic_name", "/detected_step");
    this->declare_parameter<std::string>("detected_steps_frame",      "/base_link");
    this->declare_parameter<double>("kalman_model_d0",                0.001);
    this->declare_parameter<double>("kalman_model_a0",                0.001);
    this->declare_parameter<double>("kalman_model_f0",                0.001);
    this->declare_parameter<double>("kalman_model_p0",                0.001);
    this->declare_parameter<bool>("plot_leg_kalman",                  false);
    this->declare_parameter<bool>("plot_leg_clusters",                false);
    this->declare_parameter<bool>("use_scan_header_stamp_for_tfs",    false);
    this->declare_parameter<std::vector<double>>("fixed_frame_active_area_x", 
                std::vector<double>({-0.75, 0.4}));
    this->declare_parameter<std::vector<double>>("fixed_frame_active_area_y", 
                std::vector<double>({-0.4, 0.4}));


    this->get_parameter("scan_topic",                        scan_topic_);
    this->get_parameter("detected_steps_topic_name",         detected_steps_topic_name_);
    this->get_parameter("detected_steps_frame",              detected_steps_frame_);
    this->get_parameter("kalman_model_d0_",                  kalman_model_d0_);
    this->get_parameter("kalman_model_a0_",                  kalman_model_a0_);
    this->get_parameter("kalman_model_f0_",                  kalman_model_f0_);
    this->get_parameter("kalman_model_p0_",                  kalman_model_p0_);
    this->get_parameter("plot_leg_kalman",                   plot_leg_kalman_);
    this->get_parameter("plot_leg_clusters",                 plot_leg_clusters_);
    this->get_parameter("use_scan_header_stamp_for_tfs",     use_scan_header_stamp_for_tfs_);
    this->get_parameter("fixed_frame_active_area_x", act_a_x_);
    this->get_parameter("fixed_frame_active_area_y", act_a_y_);

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
        RCLCPP_INFO(this->get_logger(), "scan_topic: [%s]", scan_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "detected_steps_topic_name: [%s]", detected_steps_topic_name_.c_str());
        RCLCPP_INFO(this->get_logger(), "detected_steps_frame: [%s]", detected_steps_frame_.c_str());
        RCLCPP_INFO(this->get_logger(), "active area:  [%.2f, %.2f] , [%.2f, %.2f] ", 
                            act_a_x_[0], act_a_x_[1], act_a_y_[0], act_a_y_[1]);        
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
    this->scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(scan_topic_, default_qos, std::bind(&KMDetectSteps::laserCallback, this, std::placeholders::_1));

}

void KMDetectSteps::laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan)  {
    walker_msgs::msg::StepStamped left_detected_step;
    walker_msgs::msg::StepStamped right_detected_step;

    // some debug info about the received scans
    if (is_debug){
        //RCLCPP_DEBUG(this->get_logger(), "%d clusters with distances bigger than %3.3f", processor.size(), cluster_dist_euclid_); 
    }


    // Find out which timestamp should be used for tfs
    bool transform_available;
    rclcpp::Time tf_time;

    // Use time from scan header
    if (use_scan_header_stamp_for_tfs_){
        tf_time = scan->header.stamp;
        try {
            buffer_->lookupTransform(detected_steps_frame_, scan->header.frame_id, tf_time, rclcpp::Duration::from_seconds(1.0));
            transform_available = buffer_->canTransform(detected_steps_frame_, scan->header.frame_id, tf_time);              
        } catch(tf2::TransformException &e) {
            RCLCPP_WARN(this->get_logger(), "No tf available");
            transform_available = false;
            
        }
    } else {
        // Otherwise just use the latest tf available
        transform_available = buffer_->canTransform(detected_steps_frame_, scan->header.frame_id, tf_time);
    }

    if(!transform_available) {
        RCLCPP_WARN(this->get_logger(), "No TF available: step position will be extrapolated");
    } else {     

        // clear rviz 
        if (is_debug){
            delete_markers();
        }

        std::list<walker_msgs::msg::StepStamped> points = getCentroids(detected_steps_frame_, scan, buffer_);
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

std::list<walker_msgs::msg::StepStamped> KMDetectSteps::getCentroids(sensor_msgs::msg::LaserScan::SharedPtr scan, std::shared_ptr<tf2_ros::Buffer> tf_buff){
    // find centroids using kmeans
    std::list<walker_msgs::msg::StepStamped> centroids;
    
    //Set area of interest
    double range;
    bool is_valid;
    geometry_msgs::msg::PointStamped laser_point;
    geometry_msgs::msg::PointStamped position_new;
    std::vector<double> laser_x;
    std::vector<double> laser_y;


    for (unsigned long int i = 0; i < scan->ranges.size(); i++){
        range = scan->ranges[i];
        is_valid  = (range > scan->range_min && range < scan->range_max);
    
        if (is_valid){
            laser_point.header = scan->header;
            laser_point.point.x = cos( scan->angle_min + i * scan->angle_increment ) * range;
            laser_point.point.y = sin( scan->angle_min + i * scan->angle_increment ) * range;
            
            // transform    
            try {
                buffer_->transform(laser_point, position_new, fixed_frame_);
                is_valid  = ( position_new.point.x >= act_a_x_[0] ) && ( position_new.point.x <= act_a_x_[1] );
                is_valid &= ( position_new.point.y >= act_a_y_[0] ) && ( position_new.point.y <= act_a_y_[1] );                    
            } catch (tf2::TransformException &e){
                is_valid = false;
                RCLCPP_ERROR (this->get_logger(), "Cant transform laser point [%s]", e.what());
            }
        }

        if (is_valid){ 
            laser_x.push_back(position_new.point.x);
            laser_y.push_back(position_new.point.y);
        }
        
    }

    // km ...

    // 1.- Init: cluster centroids in the middle of left/right active areas
    double lcx, lcy, rcx, rcy;
    rcx = lcx = (act_a_x_[0] + act_a_x_[1]) / 2.0;
    
    rcy = (act_a_y_[0]*0.75) + (act_a_y_[1]*0.25);
    lcy = (act_a_y_[0]*0.25) + (act_a_y_[1]*0.75);

    // 2.- Assign: assign points to nearest cluster (poses are in same frame than centroid)
    double dr,dl;
    for (unsigned long int i = 0; i < laser_x.size(); i++){
        dr = distance(laser_x[i], laser_y[i], rcx,rcy);
        dl = distance(laser_x[i], laser_y[i], lcx,lcy);

        if 

    }


    // 3.- Get new centroid

    // 4.- Points too far are removed
    
    // 5.- If changed: repeat
    
    return centroids;
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

int main(int argc, char **argv){

    rclcpp::init(argc, argv);
    auto node = std::make_shared<KMDetectSteps>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
