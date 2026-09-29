#include "walker_step_detector/detect_steps_fused.h"

#include <algorithm>

DetectStepsFused::DetectStepsFused() : Node("detect_steps_fused")
{
    this->declare_parameter<std::string>("rf_candidates_topic",  "/leg_candidates_rf");
    this->declare_parameter<std::string>("km_candidates_topic",  "/leg_candidates_km");
    this->declare_parameter<std::string>("seg_candidates_topic", "/leg_candidates_seg");
    this->declare_parameter<std::string>("detected_steps_topic_name", "/detected_step_fused");

    // Confianza fija asignada a los candidatos de km/segmentos (no tienen
    // clasificador propio, siempre publican confidence=1.0 aunque sea ruido).
    // Calibrada empiricamente = match_rate real medido en
    // eval_results/result_2026-09-25_gait_alternation.json (fraccion de sus
    // detecciones que caen sobre una pierna real): km ~81%, seg ~62-77%
    // (uso el peor lado). Recalibrar si cambian los datos de eval_results/.
    this->declare_parameter<double>("km_default_confidence",  0.81);
    this->declare_parameter<double>("seg_default_confidence", 0.65);
    this->declare_parameter<double>("candidates_freshness_s", 0.3);

    this->declare_parameter<double>("detection_threshold", -1.0);
    this->declare_parameter<int>("max_detected_clusters",  -1);

    this->declare_parameter<double>("kalman_model_d0", 0.001);
    this->declare_parameter<double>("kalman_model_a0", 0.001);
    this->declare_parameter<double>("kalman_model_f0", 0.001);
    this->declare_parameter<double>("kalman_model_p0", 0.001);
    this->declare_parameter<double>("max_association_dist", 0.5);
    this->declare_parameter<int>("max_track_loss_frames", 15);
    this->declare_parameter<bool>("is_debug", false);

    this->get_parameter("rf_candidates_topic",  rf_topic_);
    this->get_parameter("km_candidates_topic",  km_topic_);
    this->get_parameter("seg_candidates_topic", seg_topic_);
    this->get_parameter("detected_steps_topic_name", detected_steps_topic_name_);
    this->get_parameter("km_default_confidence",  km_default_confidence_);
    this->get_parameter("seg_default_confidence", seg_default_confidence_);
    this->get_parameter("candidates_freshness_s", candidates_freshness_s_);
    this->get_parameter("detection_threshold", detection_threshold_);
    this->get_parameter("max_detected_clusters", max_detected_clusters_);
    this->get_parameter("kalman_model_d0", kalman_model_d0_);
    this->get_parameter("kalman_model_a0", kalman_model_a0_);
    this->get_parameter("kalman_model_f0", kalman_model_f0_);
    this->get_parameter("kalman_model_p0", kalman_model_p0_);
    this->get_parameter("max_association_dist", max_association_dist_);
    this->get_parameter("max_track_loss_frames", max_track_loss_frames_);
    this->get_parameter("is_debug", is_debug_);

    kalman_tracker_.init(this, kalman_model_d0_, kalman_model_a0_, kalman_model_f0_, kalman_model_p0_,
                          max_association_dist_, max_track_loss_frames_);
    kalman_tracker_.set_status(true);

    if (is_debug_) {
        kalman_tracker_.enable_log();
        auto ret = rcutils_logging_set_logger_level(this->get_logger().get_name(), RCUTILS_LOG_SEVERITY_DEBUG);
        if (ret != RCUTILS_RET_OK) {
            RCLCPP_ERROR(this->get_logger(), "Error setting severity: %s", rcutils_get_error_string().str);
            rcutils_reset_error();
        }
        RCLCPP_INFO(this->get_logger(), "rf_candidates_topic: [%s]", rf_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "km_candidates_topic: [%s] (confidence=%.2f)", km_topic_.c_str(), km_default_confidence_);
        RCLCPP_INFO(this->get_logger(), "seg_candidates_topic: [%s] (confidence=%.2f)", seg_topic_.c_str(), seg_default_confidence_);
        RCLCPP_INFO(this->get_logger(), "candidates_freshness_s: %.2f", candidates_freshness_s_);
        RCLCPP_INFO(this->get_logger(), "detection_threshold: %.2f", detection_threshold_);
        RCLCPP_INFO(this->get_logger(), "max_detected_clusters: %d", max_detected_clusters_);
    } else {
        RCLCPP_INFO(this->get_logger(), "Fused step detector loading. Set is_debug to true for debug.");
    }

    auto default_qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());

    left_detected_step_pub_ = this->create_publisher<walker_msgs::msg::StepStamped>(detected_steps_topic_name_ + "_left", 20);
    right_detected_step_pub_ = this->create_publisher<walker_msgs::msg::StepStamped>(detected_steps_topic_name_ + "_right", 20);
    fallback_active_pub_ = this->create_publisher<std_msgs::msg::Bool>(detected_steps_topic_name_ + "_fallback_active", 20);

    rf_sub_ = this->create_subscription<walker_msgs::msg::StepArray>(
        rf_topic_, default_qos, std::bind(&DetectStepsFused::rf_callback, this, std::placeholders::_1));
    km_sub_ = this->create_subscription<walker_msgs::msg::StepArray>(
        km_topic_, default_qos, std::bind(&DetectStepsFused::km_callback, this, std::placeholders::_1));
    seg_sub_ = this->create_subscription<walker_msgs::msg::StepArray>(
        seg_topic_, default_qos, std::bind(&DetectStepsFused::seg_callback, this, std::placeholders::_1));
}

void DetectStepsFused::km_callback(const walker_msgs::msg::StepArray::SharedPtr msg)
{
    km_cache_.msg = *msg;
    km_cache_.stamp = this->now();
    km_cache_.has_data = true;
}

void DetectStepsFused::seg_callback(const walker_msgs::msg::StepArray::SharedPtr msg)
{
    seg_cache_.msg = *msg;
    seg_cache_.stamp = this->now();
    seg_cache_.has_data = true;
}

void DetectStepsFused::rf_callback(const walker_msgs::msg::StepArray::SharedPtr msg)
{
    rf_seen_ = true;
    fuse_and_publish(msg);
}

std::list<walker_msgs::msg::StepStamped> DetectStepsFused::to_candidates(
    const walker_msgs::msg::StepArray & arr, double override_confidence, bool use_own_confidence)
{
    std::list<walker_msgs::msg::StepStamped> out;
    for (const auto & s : arr.steps) {
        walker_msgs::msg::StepStamped step;
        step.position.header = arr.header;
        step.position.point = s.position;
        step.confidence = use_own_confidence ? s.confidence : static_cast<float>(override_confidence);
        step.tracked = s.tracked;
        step.speed = s.speed;
        step.load = s.load;
        out.push_back(step);
    }
    return out;
}

void DetectStepsFused::fuse_and_publish(const walker_msgs::msg::StepArray::SharedPtr rf_msg)
{
    std::list<walker_msgs::msg::StepStamped> points = to_candidates(*rf_msg, 0.0, true);

    // RF primero: solo se recurre a km/segmentos como RESPALDO, cuando RF no
    // dio suficientes candidatos utiles este ciclo (huecos por oclusion o
    // fallo puntual del clasificador) -- nunca para competir con una
    // deteccion de RF ya buena. Probado sin esta prioridad (siempre se
    // anadian los tres): la fusion quedaba POR DEBAJO de RF solo (ver
    // eval_results/result_2026-09-25_fused.json, 47 bags) porque el
    // emparejamiento por distancia de legs_tracker.cpp no pondera por
    // confianza -- un candidato ruidoso de km/seg podia "ganar" el
    // emparejamiento por estar un poco mas cerca en ese instante, sin
    // penalizacion por venir de una fuente menos fiable.
    size_t rf_usable = std::count_if(points.begin(), points.end(),
        [this](const walker_msgs::msg::StepStamped & s) { return s.confidence >= detection_threshold_; });

    bool fallback_active = (rf_usable < 2);
    if (fallback_active) {
        rclcpp::Time now = this->now();
        if (km_cache_.has_data && (now - km_cache_.stamp).seconds() <= candidates_freshness_s_) {
            auto km_points = to_candidates(km_cache_.msg, km_default_confidence_, false);
            points.splice(points.end(), km_points);
        }
        if (seg_cache_.has_data && (now - seg_cache_.stamp).seconds() <= candidates_freshness_s_) {
            auto seg_points = to_candidates(seg_cache_.msg, seg_default_confidence_, false);
            points.splice(points.end(), seg_points);
        }
    }
    std_msgs::msg::Bool fallback_msg;
    fallback_msg.data = fallback_active;
    fallback_active_pub_->publish(fallback_msg);

    // Mismo filtrado que detect_steps.cpp::getCentroids: descarta candidatos
    // de baja confianza y, si sobran, se queda con los mas confiables (no con
    // los primeros que aparecieron).
    points.remove_if([this](const walker_msgs::msg::StepStamped & s) {
        return s.confidence < detection_threshold_;
    });
    if (max_detected_clusters_ > 0 && static_cast<int>(points.size()) > max_detected_clusters_) {
        points.sort([](const walker_msgs::msg::StepStamped & a, const walker_msgs::msg::StepStamped & b) {
            return a.confidence > b.confidence;
        });
        points.resize(max_detected_clusters_);
    }

    if (is_debug_) {
        RCLCPP_DEBUG(this->get_logger(), "Fusionando %ld candidatos (tras filtro)", points.size());
    }

    kalman_tracker_.add_detections(points);

    walker_msgs::msg::StepStamped step_r, step_l;
    double t = this->now().nanoseconds();
    kalman_tracker_.get_steps(&step_r, &step_l, t);

    if (kalman_tracker_.is_init) {
        if (step_r.position.header.frame_id.compare("invalid") != 0) {
            right_detected_step_pub_->publish(step_r);
        }
        if (step_l.position.header.frame_id.compare("invalid") != 0) {
            left_detected_step_pub_->publish(step_l);
        }
    }
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DetectStepsFused>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
