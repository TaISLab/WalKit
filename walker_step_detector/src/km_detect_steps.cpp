#include "walker_step_detector/km_detect_steps.h"

KMDetectSteps::KMDetectSteps() : Node("detect_steps"){
    //Get ROS parameters
    this->declare_parameter<std::string>("scan_topic",                "/scan");
    this->declare_parameter<std::string>("detected_steps_topic_name", "/detected_step");
    this->declare_parameter<std::string>("detected_steps_frame",      "/base_link");
    this->declare_parameter<bool>("fit_ellipse",                      false);
    this->declare_parameter<bool>("kalman_enabled",                   false);
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
    this->get_parameter("fit_ellipse",                      fit_ellipse_);
    this->get_parameter("kalman_enabled",                   kalman_enabled_);
    this->get_parameter("kalman_model_d0",                  kalman_model_d0_);
    this->get_parameter("kalman_model_a0",                  kalman_model_a0_);
    this->get_parameter("kalman_model_f0",                  kalman_model_f0_);
    this->get_parameter("kalman_model_p0",                  kalman_model_p0_);
    this->get_parameter("plot_leg_kalman",                   plot_leg_kalman_);
    this->get_parameter("plot_leg_clusters",                 plot_leg_clusters_);
    this->get_parameter("use_scan_header_stamp_for_tfs",     use_scan_header_stamp_for_tfs_);
    this->get_parameter("fixed_frame_active_area_x", act_a_x_);
    this->get_parameter("fixed_frame_active_area_y", act_a_y_);

    // WARNING: tf2 frame_ids cannot start with a '/'
    if (detected_steps_frame_.rfind("/", 0) == 0) { // pos=0 limits the search to the prefix
        detected_steps_frame_.erase(0, 1);
    }

    // Load kalman tracker
    kalman_tracker.init(this, kalman_model_d0_, kalman_model_a0_, kalman_model_f0_, kalman_model_p0_ );
    kalman_tracker.set_status(kalman_enabled_);

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
        filter_laser_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("active_laser", 20);
    }

    // transform buffer
    buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tfl_ = std::make_shared<tf2_ros::TransformListener>(*buffer_);
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
                                                        this->get_node_base_interface(),
                                                        this->get_node_timers_interface());
    buffer_->setCreateTimerInterface(timer_interface);

    //subscribers last
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

        std::list<walker_msgs::msg::StepStamped> points = getCentroids(scan);



        kalman_tracker.add_detections(points);
    }

    // get steps from Kalman set
    walker_msgs::msg::StepStamped step_r;
    walker_msgs::msg::StepStamped step_l;
    double t = (this->now()).nanoseconds();

    bool got_steps = kalman_tracker.get_steps(&step_r, &step_l, t);

    if (got_steps){

        // publish lets
        if (kalman_tracker.is_init){
            right_detected_step_pub_->publish(step_r);
            left_detected_step_pub_->publish(step_l);
        }

        if (is_debug){
            publish_leg(step_r, 0);
            publish_leg(step_l, 1);
            publish_active_area();
            RCLCPP_DEBUG(this->get_logger(), ".......\n\n"); 
        }
    }else{
        RCLCPP_WARN(this->get_logger(), "No valid clusters found, skipping");
    }
}

std::list<walker_msgs::msg::StepStamped> KMDetectSteps::getCentroids(sensor_msgs::msg::LaserScan::SharedPtr scan){
    sensor_msgs::msg::LaserScan scan_filtered;


    scan_filtered.header = scan->header;
    scan_filtered.angle_min = scan->angle_min;
    scan_filtered.angle_max = scan->angle_max;
    scan_filtered.angle_increment = scan->angle_increment;
    scan_filtered.time_increment = scan->time_increment;
    scan_filtered.scan_time = scan->scan_time;
    scan_filtered.range_min = scan->range_min; 
    scan_filtered.range_max = scan->range_max;

    // find centroids using kmeans
    std::list<walker_msgs::msg::StepStamped> centroids;
    
    //Set area of interest
    double range;
    bool is_valid;
    geometry_msgs::msg::PointStamped laser_point;
    geometry_msgs::msg::PointStamped position_new;
    std::vector<double> laser_x;
    std::vector<double> laser_y;

    // get laser data in cartesians
    for (unsigned long int i = 0; i < scan->ranges.size(); i++){
        range = scan->ranges[i];
        is_valid  = (range > scan->range_min && range < scan->range_max);
    
        if (is_valid){
            laser_point.header = scan->header;
            laser_point.point.x = cos( scan->angle_min + i * scan->angle_increment ) * range;
            laser_point.point.y = sin( scan->angle_min + i * scan->angle_increment ) * range;
            
            // transform    
            try {
                buffer_->transform(laser_point, position_new, detected_steps_frame_);
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
            scan_filtered.ranges.push_back(scan->ranges[i]);
        } else {
            scan_filtered.ranges.push_back(scan->range_max + 1);            
        }
        
    }
    if (is_debug){
     filter_laser_pub_->publish(scan_filtered);
    }

    if (laser_x.size()>1){

        // km ...
        int attempts, max_attempts;
        double max_d;
        double dr,dl;
        double lcx, lcy, rcx, rcy;
        bool has_moved = true;
            std::vector<unsigned int> r_points;
            std::vector<double> r_dists;
            std::vector<unsigned int> l_points;
            std::vector<double> l_dists;

        // COW any point further than this from centroid, won't be used
        // COW three km iterations
        max_d = 0.5;
        max_attempts = 3;

        // Init: cluster centroids in the middle of left/right active areas
        rcx = lcx = (act_a_x_[0] + act_a_x_[1]) / 2.0;
        
        rcy = (act_a_y_[0]*0.75) + (act_a_y_[1]*0.25);
        lcy = (act_a_y_[0]*0.25) + (act_a_y_[1]*0.75);
        attempts = 0;

        //RCLCPP_INFO(this->get_logger(), "(%d): \n\tright centroid:\t[%.2f, %.2f] \n\tleft centroid\t[%.2f, %.2f] ", 
        //                        attempts, rcx, rcy, lcx, lcy);   
        while ((has_moved) & (attempts<max_attempts)){
            attempts = attempts + 1;
            // Assign: assign points to nearest cluster (poses are in same frame than centroid)

            bool goodSplit = false;
            double offset = 0;

            while (!goodSplit){
                r_points.clear();
                r_dists.clear();
                l_points.clear();
                l_dists.clear();

                for (unsigned long int i = 0; i < laser_x.size(); i++){
                    dr = distance(laser_x[i], laser_y[i], rcx, rcy);
                    dl = distance(laser_x[i], laser_y[i], lcx, lcy);

                    if (dr<dl+offset){
                        // assign to right cluster
                        r_points.push_back(i);
                        r_dists.push_back(dr);
                    } else if (dr>dl+offset){
                        // assign to left cluster
                        l_points.push_back(i);
                        l_dists.push_back(dl);            
                    }
                }

                goodSplit = (r_points.size()>0) & (l_points.size()>0);
                if (r_points.size()==0){
                    offset = offset + 0.01;
                }
                if (l_points.size()==0){
                    offset = offset - 0.01;                    
                }                
            }

            // Get new centroid: far points are skipped from centroid
            auto [rcx_new, rcy_new] = find_centroid(laser_x, laser_y, r_points, r_dists, max_d);
            auto [lcx_new, lcy_new] = find_centroid(laser_x, laser_y, l_points, l_dists, max_d);

            // If changed: repeat, but no more than n times
            has_moved = ( distance(rcx_new, rcy_new, rcx, rcy) > 0.05 ) |
                        ( distance(lcx_new, lcy_new, lcx, lcy) > 0.05 );

            // update values
            rcx = rcx_new; 
            rcy = rcy_new; 
            lcx = lcx_new; 
            lcy = lcy_new; 
            //RCLCPP_INFO(this->get_logger(), "(%d): \n\tright centroid:\t[%.2f, %.2f] \n\tleft centroid\t[%.2f, %.2f] ", 
            //                    attempts, rcx, rcy, lcx, lcy);   
        }

        if (fit_ellipse_){
            RCLCPP_INFO(this->get_logger(), "Fitting result to ellipse ");   

            double r_phi;
            double r_width;
            double r_hight;
            fit_ellipse(rcx, rcy, r_phi, r_width, r_hight, laser_x, laser_y, r_points);

            double l_phi;
            double l_width;
            double l_hight;
            fit_ellipse(lcx, lcy, l_phi, l_width, l_hight, laser_x, laser_y, l_points);
        }

        walker_msgs::msg::StepStamped r_step;
        r_step.position.header = laser_point.header; // sime time
        r_step.position.header.frame_id = detected_steps_frame_; // different frame_id
        r_step.position.point.x = rcx;
        r_step.position.point.y = rcy;
        r_step.confidence = 1;  // COW ! TODO!
        centroids.push_back(r_step);

        walker_msgs::msg::StepStamped l_step;
        l_step.position.header = laser_point.header; // sime time
        l_step.position.header.frame_id = detected_steps_frame_; // different frame_id
        l_step.position.point.x = lcx;
        l_step.position.point.y = lcy;
        l_step.confidence = 1;  // COW ! TODO!
        centroids.push_back(l_step);
    
    }

    return centroids;
}

double KMDetectSteps::distance(double ax, double ay, double bx, double by){
    return std::sqrt(std::pow(ax - bx, 2) + std::pow(ay - by, 2) );
}

std::tuple<double, double> KMDetectSteps::find_centroid(std::vector<double>& x, 
                                  std::vector<double>& y,
                                  std::vector<unsigned int>& selected_indexs,
                                  std::vector<double>& selected_dists, 
                                  double max_d) {

   
    double cx = 0;
    double cy = 0;
    unsigned int used_points = 0;
    unsigned int i;
    double d;
    bool keepGoing;
    double y_min;
    double y_max;

    keepGoing = selected_indexs.size()>0;
    //RCLCPP_INFO(this->get_logger(), "Searchin centroid of (%ld) points ", selected_indexs.size()); 
    while (keepGoing){
        y_min=1000;
        y_max=0;
        for (unsigned long int it = 0; it < selected_indexs.size(); it++){
            i = selected_indexs[it];
            d = selected_dists[it];
            // Points too far are not used: avoid points far from previous centroid
            if (d<max_d) {
                cx += x[i];
                cy += y[i];
                used_points = used_points + 1; 
            }

            if (y[i]<y_min){
                y_min = y[i];
            }
            if (y[i]>y_max){
                y_max = y[i];
            }

        }

        if (used_points == 0){
            max_d = 1.1*max_d;
            
        } else {
            keepGoing = false;
        }
    }
    //RCLCPP_INFO(this->get_logger(), "Used (%d) points ", used_points); 
    cx = cx/used_points;
    cy = cy/used_points;

    cx = cx - (y_max-y_min)/2.0;

    return  {cx, cy};
}

void KMDetectSteps::delete_markers(){        
        visualization_msgs::msg::Marker marker_cluster;
        visualization_msgs::msg::MarkerArray marker_array;

        marker_cluster.action = visualization_msgs::msg::Marker::DELETEALL;
        marker_array.markers.push_back(marker_cluster);

        markers_array_pub_->publish(marker_array);
}

void KMDetectSteps::publish_active_area(){
    visualization_msgs::msg::MarkerArray marker_array;
    visualization_msgs::msg::Marker line_strip;
    line_strip.header.frame_id = detected_steps_frame_;
    line_strip.header.stamp = this->get_clock()->now();
    line_strip.ns  = "points_and_lines";
    line_strip.action = visualization_msgs::msg::Marker::ADD;
    line_strip.pose.orientation.w = 1.0;
    line_strip.id = 1;

    line_strip.type = visualization_msgs::msg::Marker::LINE_STRIP;

    // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
    line_strip.scale.x = 0.05;

    // Line strip is half transparent blue 
    line_strip.color.b = 1.0;
    line_strip.color.a = 0.5;

    // // Create the vertices for the points and lines    
    geometry_msgs::msg::Point p1,p2,p3,p4;
    p1.x = act_a_x_[0];
    p1.y = act_a_y_[0];
    p1.z = 0;
    line_strip.points.push_back(p1);

    p2.x = act_a_x_[0];
    p2.y = act_a_y_[1];
    p2.z = 0;
    line_strip.points.push_back(p2);

    p3.x = act_a_x_[1];
    p3.y = act_a_y_[1];
    p3.z = 0;
    line_strip.points.push_back(p3);

    p4.x = act_a_x_[1];
    p4.y = act_a_y_[0];
    p4.z = 0;
    line_strip.points.push_back(p4);
    line_strip.points.push_back(p1);

    marker_array.markers.push_back(line_strip);
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

/*
void KMDetectSteps::fit_ellipse(double& result_center_x, double& result_center_y, 
                      double& result_phi, double& result_width, double& result_hight,
                      std::vector<double>& x, std::vector<double>& y, std::vector<unsigned int>& selected_indexs){

    size_t size_mat = selected_indexs.size();

    if (size_mat<2){
        result_center_x = x[0];
        result_center_y = y[0];
        result_width = 0;
        result_hight = 0;
        result_phi = 0;
    } else {
        RCLCPP_INFO(this->get_logger(), "Ellipse from (%ld) points ", size_mat); 
        Eigen::VectorXd X_val(size_mat);
        Eigen::VectorXd Y_val(size_mat);
        unsigned int k;

        for (unsigned long int it = 0; it < selected_indexs.size(); it++){
            k = selected_indexs[it];
            X_val(it)= x[k];
            Y_val(it)= y[k];
        }

        Eigen::VectorXd D1_col0(size_mat);
        Eigen::VectorXd D1_col1(size_mat);
        Eigen::VectorXd D1_col2(size_mat);

        D1_col0 = X_val.array().pow(2);
        D1_col1 = X_val.array() * Y_val.array();
        D1_col2 = Y_val.array().pow(2);


        Eigen::MatrixXd D1(size_mat,3);

        D1.col(0) = D1_col0;
        D1.col(1) = D1_col1;
        D1.col(2) = D1_col2;

        Eigen::MatrixXd D2(size_mat,3);

        D2.col(0) = X_val;
        D2.col(1) = Y_val;
        D2.col(2) = Eigen::VectorXd::Ones(size_mat);


        Eigen::MatrixXd S1(3,3);
        Eigen::MatrixXd S2(3,3);
        Eigen::MatrixXd S3(3,3);


        S1 = D1.transpose() * D1;
        S2 = D1.transpose() * D2;
        S3 = D2.transpose() * D2;

        RCLCPP_INFO(this->get_logger(), "ALIVE HERE! (%d) ",__LINE__); 

        Eigen::MatrixXd C1(3,3);
        C1<<0,0,2,0,-1,0,2,0,0;

        Eigen::MatrixXd M;

        M= C1.inverse()* (S1 - S2*S3.inverse()*S2.transpose());
        RCLCPP_INFO(this->get_logger(), "ALIVE HERE! (%d) ",__LINE__); 

        Eigen::EigenSolver<MatrixXd> s(M);


        MatrixXd eigenvector= s.eigenvectors().real();


        Eigen::VectorXd eig_row0 = eigenvector.row(0);
        Eigen::VectorXd eig_row1 = eigenvector.row(1);
        Eigen::VectorXd eig_row2 = eigenvector.row(2);

        Eigen::VectorXd cond = 4* (eig_row0.array() * eig_row2.array()) - eig_row1.array().pow(2);
        RCLCPP_INFO(this->get_logger(), "ALIVE HERE! (%d) ",__LINE__); 

        Eigen::VectorXd min_pos_eig;

        for(int i= 0; i<3 ; i++){
            if(cond(i) >0){
                min_pos_eig = eigenvector.col(i);
                break;
            }
        }

        RCLCPP_INFO(this->get_logger(), "ALIVE HERE! (%d) ",__LINE__); 

        Eigen::VectorXd coeffs(6);

        if (S3.determinant()==0){
            RCLCPP_INFO(this->get_logger(), "ABOUT TO DIE.................... "); 
        }
        RCLCPP_INFO(this->get_logger(), "S3 det is (%3.3f) ",S3.determinant()); 

        Eigen::VectorXd cont_matrix =  S3.inverse();


        RCLCPP_INFO(this->get_logger(), "ALIVE HERE! (%d) ",__LINE__); 
        cont_matrix =  -1 * cont_matrix;
        RCLCPP_INFO(this->get_logger(), "ALIVE HERE! (%d) ",__LINE__); 
        cont_matrix =  cont_matrix * S2.transpose();
        RCLCPP_INFO(this->get_logger(), "ALIVE HERE! (%d) ",__LINE__); 
        cont_matrix =  cont_matrix * min_pos_eig;

        coeffs<<min_pos_eig, cont_matrix;
        RCLCPP_INFO(this->get_logger(), "ALIVE HERE! (%d) ",__LINE__); 

        double a = coeffs(0);
        double b = coeffs(1)/2;
        double c = coeffs(2);
        double d = coeffs(3)/2;
        double f = coeffs(4)/2;
        double g = coeffs(5);

        double center_x = (c*d-b*f)/(pow(b,2)-a*c);
        double center_y = (a*f-b*d)/(pow(b,2)-a*c);

        double numerator = 2*(a*f*f+c*d*d+g*b*b-2*b*d*f-a*c*g);
        double denominator1 = (b*b-a*c)*( (c-a)*sqrt(1+4*b*b/((a-c)*(a-c)))-(c+a));
        double denominator2 = (b*b-a*c)*( (a-c)*sqrt(1+4*b*b/((a-c)*(a-c)))-(c+a));
        double width = sqrt(numerator/denominator1);
        double height = sqrt(numerator/denominator2);
        double phi = 0.5*atan((2*b)/(a-c));


        result_center_x = center_x;
        result_center_y = center_y;
        result_width = width;
        result_hight = height;
        result_phi = phi;
    }
}
*/

void KMDetectSteps::fit_ellipse(double& result_center_x, double& result_center_y, 
                      double& result_phi, double& result_width, double& result_hight,
                      std::vector<double>& x, std::vector<double>& y, std::vector<unsigned int>& selected_indexs){

    size_t n_points = selected_indexs.size();

        RCLCPP_INFO(this->get_logger(), "Ellipse from (%ld) points ", n_points); 
        std::vector<cv::Point2f> pts(n_points);

        unsigned int k;

        for (unsigned long int it = 0; it < n_points; it++){
            k = selected_indexs[it];
            pts[it] = cv::Point2f((float) x[k], (float) y[k]);
        }

    if (n_points<5){
        cv::Point2f center;
        float radius;
        cv::minEnclosingCircle(pts, center, radius);

        result_center_x = center.x;
        result_center_y = center.y;
        result_width = radius;
        result_hight = radius;
        result_phi = 1;
    } else {
        cv::RotatedRect box = cv::fitEllipse(pts);  //  fitEllipseAMS  // fitEllipseDirect
        /*
        double xc    = box.center.x;
        double yc    = box.center.y;
        double a     = box.size.width  / 2;    // width >= height
        double b     = box.size.height / 2;
        double theta = box.angle;              // in degrees
        */
        result_center_x = box.center.x;
        result_center_y= box.center.y;
    }        



}

int main(int argc, char **argv){

    rclcpp::init(argc, argv);
    auto node = std::make_shared<KMDetectSteps>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
