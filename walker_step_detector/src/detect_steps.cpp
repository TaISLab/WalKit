#include "walker_step_detector/detect_steps.h"

DetectSteps::DetectSteps() : Node("detect_steps"){
    //Get ROS parameters
    this->declare_parameter<std::string>("scan_topic",                "/scan");
    this->declare_parameter<std::string>("forest_file",               "./src/leg_detector/config/trained_leg_detector_res_0.33.yaml");
    this->declare_parameter<std::string>("detected_steps_topic_name", "/detected_step");
    this->declare_parameter<std::string>("candidates_topic_name",     "/leg_candidates_rf");
    this->declare_parameter<double>("kalman_model_d0",                0.001);
    this->declare_parameter<double>("kalman_model_a0",                0.001);
    this->declare_parameter<double>("kalman_model_f0",                0.001);
    this->declare_parameter<double>("kalman_model_p0",                0.001);
    this->declare_parameter<double>("max_association_dist",           0.5);
    this->declare_parameter<int>("max_track_loss_frames",             15);
    this->declare_parameter<double>("detection_threshold",            -1.0);
    this->declare_parameter<double>("cluster_dist_euclid",            0.13);
    this->declare_parameter<double>("max_detect_distance",            10.0);
    this->declare_parameter<int>("max_detected_clusters",             -1);
    this->declare_parameter<int>("min_points_per_cluster",            3);    
    this->declare_parameter<bool>("publish_clusters",                 false);

    this->get_parameter("scan_topic",                    scan_topic_name_);
    this->get_parameter("forest_file",                   forest_file);
    this->get_parameter("detected_steps_topic_name",     detected_steps_topic_name_);
    this->get_parameter("candidates_topic_name",         candidates_topic_name_);
    this->get_parameter("kalman_model_d0",                  kalman_model_d0_);
    this->get_parameter("kalman_model_a0",                  kalman_model_a0_);
    this->get_parameter("kalman_model_f0",                  kalman_model_f0_);
    this->get_parameter("kalman_model_p0",                  kalman_model_p0_);
    this->get_parameter("max_association_dist",             max_association_dist_);
    this->get_parameter("max_track_loss_frames",             max_track_loss_frames_);
    this->get_parameter("detection_threshold",           detection_threshold_);
    this->get_parameter("cluster_dist_euclid",           cluster_dist_euclid_);
    this->get_parameter("max_detect_distance",           max_detect_distance_);
    this->get_parameter("max_detected_clusters",         max_detected_clusters_);
    this->get_parameter("min_points_per_cluster",        min_points_per_cluster_);
    this->get_parameter("publish_clusters",              publish_clusters_);

    // Load kalman tracker
    kalman_tracker.init(this, kalman_model_d0_, kalman_model_a0_, kalman_model_f0_, kalman_model_p0_, max_association_dist_, max_track_loss_frames_ );
    kalman_tracker.set_status(true);

    // Verbose init
    is_debug = (publish_clusters_);

    if (is_debug){
        kalman_tracker.enable_log();

        auto ret = rcutils_logging_set_logger_level( this->get_logger().get_name(), RCUTILS_LOG_SEVERITY_DEBUG);
        if (ret != RCUTILS_RET_OK) {
            RCLCPP_ERROR(this->get_logger(), "Error setting severity: [%s]", rcutils_get_error_string().str);
            rcutils_reset_error();
        }

        //Printing ROS parameters
        RCLCPP_INFO(this->get_logger(), "scan_topic: [%s]", scan_topic_name_.c_str());
        RCLCPP_INFO(this->get_logger(), "forest_file: [%s]", forest_file.c_str());
        RCLCPP_INFO(this->get_logger(), "detected_steps_topic_name: [%s]", detected_steps_topic_name_.c_str());
        RCLCPP_INFO(this->get_logger(), "kalman model: ");
        RCLCPP_INFO(this->get_logger(), "       - d: %.2f", kalman_model_d0_);
        RCLCPP_INFO(this->get_logger(), "       - a: %.2f", kalman_model_a0_);
        RCLCPP_INFO(this->get_logger(), "       - f: %.2f", kalman_model_f0_);
        RCLCPP_INFO(this->get_logger(), "       - p: %.2f", kalman_model_p0_);
        RCLCPP_INFO(this->get_logger(), "max_association_dist: %.2f", max_association_dist_);
        RCLCPP_INFO(this->get_logger(), "max_track_loss_frames: %d", max_track_loss_frames_);
        RCLCPP_INFO(this->get_logger(), "detection_threshold: %.2f", detection_threshold_);
        RCLCPP_INFO(this->get_logger(), "cluster_dist_euclid: %.2f", cluster_dist_euclid_);
        RCLCPP_INFO(this->get_logger(), "max_detect_distance: %.2f", max_detect_distance_);
        RCLCPP_INFO(this->get_logger(), "max_detected_clusters: %d", max_detected_clusters_);
        RCLCPP_INFO(this->get_logger(), "min_points_per_cluster: %d", min_points_per_cluster_);
    } else {
        RCLCPP_INFO(this->get_logger(), "Step detector loading. Set publish_clusters to true for debug.");
    }

    //Load Random forest
    processor.setForestFile(forest_file);

    //ROS STUFF
    auto default_qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());

    // publishers
    left_detected_step_pub_ = this->create_publisher<walker_msgs::msg::StepStamped>(detected_steps_topic_name_ + "_left", 20);
    right_detected_step_pub_ = this->create_publisher<walker_msgs::msg::StepStamped>(detected_steps_topic_name_ + "_right", 20);
    candidates_pub_ = this->create_publisher<walker_msgs::msg::StepArray>(candidates_topic_name_, 20);
    left_ref_pub_ = this->create_publisher<walker_msgs::msg::StepStamped>(detected_steps_topic_name_ + "_left_ref", 20);
    right_ref_pub_ = this->create_publisher<walker_msgs::msg::StepStamped>(detected_steps_topic_name_ + "_right_ref", 20);


    this->scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(scan_topic_name_, default_qos, std::bind(&DetectSteps::laserCallback, this, std::placeholders::_1));

}

void DetectSteps::laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan){
    walker_msgs::msg::StepStamped left_detected_step;
    walker_msgs::msg::StepStamped right_detected_step;

    RCLCPP_WARN(this->get_logger(), "Laser data received at: [%3.3f]", scan->header.stamp.sec + (scan->header.stamp.nanosec*1e-9));    

    std::list<walker_msgs::msg::StepStamped> points = getCentroids(scan);
    candidates_pub_->publish(walker_step_detector::to_step_array(scan->header, points));

    // Referencia interna ANTES de procesar este ciclo (ver comentario en el
    // .h) -- se publica aqui, no despues de add_detections, para capturar
    // exactamente lo que la asociacion de este ciclo usara como l_ref/r_ref.
    walker_msgs::msg::StepStamped left_ref, right_ref;
    kalman_tracker.get_current_refs(&left_ref, &right_ref);
    left_ref_pub_->publish(left_ref);
    right_ref_pub_->publish(right_ref);

    RCLCPP_DEBUG(this->get_logger(), "Adding centroid detections to kalman filters");
    kalman_tracker.add_detections(points);
    
    // get steps from Kalman set
    RCLCPP_DEBUG(this->get_logger(), "Getting filtered positions from kalman");
    walker_msgs::msg::StepStamped step_r;
    walker_msgs::msg::StepStamped step_l;
    // Timestamp del propio escaner, NO this->now() -- mismo bug y mismo
    // fix que km_detect_steps.cpp::laserCallback (ver ese comentario):
    // invisible en vivo, pero trunca Tr/SpT/SdT a ~1/rate durante replay
    // de bag a una tasa distinta de 1.0. No afecta a la precision de
    // POSICION (por eso el 95.7-95.9% de match_rate medido con
    // eval_against_mocap.py, que compara solo posicion, no se vio
    // afectado), solo a cualquier metrica derivada del tiempo entre pasos.
    double t = rclcpp::Time(scan->header.stamp).nanoseconds();
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



std::list<walker_msgs::msg::StepStamped> DetectSteps::getCentroids(sensor_msgs::msg::LaserScan::SharedPtr scan){
        processor.setScan(*scan);
        processor.splitConnected(cluster_dist_euclid_);

        if (is_debug){
            RCLCPP_DEBUG(this->get_logger(), "Detected %d clusters with distances bigger than %3.3f", processor.size(), cluster_dist_euclid_); 
        }

        processor.removeLessThan(min_points_per_cluster_);

        if (is_debug){
            RCLCPP_DEBUG(this->get_logger(), "Detected %d clusters with at least %d points", processor.size(), min_points_per_cluster_); 
        }

        // Remove far clusters
        processor.removeFar(max_detect_distance_);

        if (is_debug){
            RCLCPP_DEBUG(this->get_logger(), "Detected %d clusters closer than %3.3f", processor.size(), max_detect_distance_); 
        }

        // selected clusters for legs
        if (publish_clusters_){
            //publish_clusters(processor.getClusters());
        }

        std::list<walker_msgs::msg::StepStamped> points = processor.getCentroids(scan->header);

        // Filtrar por confianza del clasificador: sin esto, detection_threshold_
        // y max_detected_clusters_ se declaraban, se leian del parametro y se
        // logueaban al arrancar, pero NUNCA se usaban para descartar nada --
        // todos los clusters (sean pierna o ruido) llegaban igual al tracker.
        // Con el umbral por defecto (-1.0) esto no filtra nada (la confianza
        // del forest vive en [0,1]), asi que no cambia el comportamiento de
        // quien no fije el parametro explicitamente.
        points.remove_if([this](const walker_msgs::msg::StepStamped & s){
            return s.confidence < detection_threshold_;
        });

        if (is_debug){
            RCLCPP_DEBUG(this->get_logger(), "Detected %ld clusters with confidence >= %3.3f", points.size(), detection_threshold_);
        }

        if (max_detected_clusters_ > 0 && (int)points.size() > max_detected_clusters_){
            // mas candidatos de los permitidos: quedarse con los N mas
            // confiables, no con los N primeros del barrido angular.
            points.sort([](const walker_msgs::msg::StepStamped & a, const walker_msgs::msg::StepStamped & b){
                return a.confidence > b.confidence;
            });
            points.resize(max_detected_clusters_);

            if (is_debug){
                RCLCPP_DEBUG(this->get_logger(), "Capped to the %d most confident clusters", max_detected_clusters_);
            }
        }

        return points;
}

int main(int argc, char **argv){

    rclcpp::init(argc, argv);
    auto node = std::make_shared<DetectSteps>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
