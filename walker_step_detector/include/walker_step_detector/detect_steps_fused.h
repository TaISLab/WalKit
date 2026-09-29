#ifndef DETECTSTEPSFUSED_HH
#define DETECTSTEPSFUSED_HH

// Fusiona los candidatos crudos de las tres variantes (detect_steps/RF,
// km_detect_steps, detect_steps_s/segmentos -- topics /leg_candidates_*,
// walker_msgs/StepArray) en un UNICO LegsTracker, en vez de tener tres
// trackers independientes. Los tres nodos originales siguen funcionando
// igual (para comparacion/depuracion, ver scripts/run_multibag_eval.py);
// este nodo es puramente aditivo, no los reemplaza ni los modifica.
//
// RF ya tiene una confianza real (0-1, el clasificador de laser_processor.cpp);
// km y segmentos no tienen clasificador propio y siempre publican confidence=1.0
// diga lo que diga la deteccion -- aqui se sustituye por una confianza fija
// calibrada empiricamente contra su tasa de acierto real medida en
// eval_results/ (ver km_default_confidence/seg_default_confidence).
//
// Disparo: se procesa un ciclo de fusion cada vez que llega un mensaje de RF
// (la variante mas fiable, ver eval_results/), incorporando el ultimo
// mensaje de km/segmentos si es lo bastante reciente (candidates_freshness_s).
// Si RF no esta publicando (no lanzado, o el bag no tiene el forest), este
// nodo simplemente no producira nada -- es una limitacion de diseño conocida,
// no un fallo silencioso: se loguea si nunca ha llegado nada de RF.

#include <rclcpp/rclcpp.hpp>

#include "std_msgs/msg/bool.hpp"
#include "walker_msgs/msg/step_array.hpp"
#include "walker_msgs/msg/step_stamped.hpp"

#include "walker_step_detector/legs_tracker.h"

#include <list>
#include <memory>
#include <string>

class DetectStepsFused : public rclcpp::Node
{
public:
    DetectStepsFused();

private:
    // ultimo StepArray recibido de cada fuente + cuando (para la ventana de
    // frescura al fusionar en el callback de RF)
    struct CachedCandidates
    {
        walker_msgs::msg::StepArray msg;
        rclcpp::Time stamp;
        bool has_data = false;
    };

    void rf_callback(const walker_msgs::msg::StepArray::SharedPtr msg);
    void km_callback(const walker_msgs::msg::StepArray::SharedPtr msg);
    void seg_callback(const walker_msgs::msg::StepArray::SharedPtr msg);

    // convierte un StepArray a StepStamped con una confianza fija (para
    // km/segmentos, que no traen una confianza real util) o la propia (RF)
    std::list<walker_msgs::msg::StepStamped> to_candidates(
        const walker_msgs::msg::StepArray & arr, double override_confidence, bool use_own_confidence);

    void fuse_and_publish(const walker_msgs::msg::StepArray::SharedPtr rf_msg);

    bool is_debug_;

    std::string rf_topic_, km_topic_, seg_topic_;
    std::string detected_steps_topic_name_;
    double km_default_confidence_;
    double seg_default_confidence_;
    double candidates_freshness_s_;
    double detection_threshold_;
    int max_detected_clusters_;

    double kalman_model_d0_, kalman_model_a0_, kalman_model_f0_, kalman_model_p0_;
    double max_association_dist_;
    int max_track_loss_frames_;

    CachedCandidates km_cache_, seg_cache_;
    bool rf_seen_ = false;

    LegsTracker kalman_tracker_;

    rclcpp::Subscription<walker_msgs::msg::StepArray>::SharedPtr rf_sub_;
    rclcpp::Subscription<walker_msgs::msg::StepArray>::SharedPtr km_sub_;
    rclcpp::Subscription<walker_msgs::msg::StepArray>::SharedPtr seg_sub_;
    rclcpp::Publisher<walker_msgs::msg::StepStamped>::SharedPtr left_detected_step_pub_;
    rclcpp::Publisher<walker_msgs::msg::StepStamped>::SharedPtr right_detected_step_pub_;

    // Marca, UNA vez por ciclo de fusion, si el respaldo de km/segmentos se
    // activo ese ciclo (RF dio menos de 2 candidatos utiles) -- para poder
    // analizar offline, sobre datos grabados, si el respaldo aporta algo
    // justo donde deberia (ver scripts/analyze_fallback_gain.py). Topic, no
    // log: un log de depuracion no queda grabado en el bag de evaluacion.
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr fallback_active_pub_;
};

#endif  // DETECTSTEPSFUSED_HH
