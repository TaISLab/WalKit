#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <vector>
#include <string>
#include <cmath>
#include <limits>

struct RectRegion {
    float min_x, max_x;
    float min_y, max_y;

    bool contains(float x, float y) const {
        return (x >= min_x && x <= max_x && y >= min_y && y <= max_y);
    }
};

class AllFilterNode : public rclcpp::Node {
public:
    AllFilterNode() : Node("all_filter_node") {
        this->declare_parameter<std::string>("scan_topic",          "scan");
        this->declare_parameter<std::string>("scan_filtered_topic", "scan_filtered");
        this->declare_parameter<std::string>("scan_nav_topic",      "scan_nav");

        this->declare_parameter<std::vector<double>>("region_nav", {-1.0, 1.0, -1.0, 1.0});
        this->declare_parameter<std::vector<double>>("regions_filtered", std::vector<double>(16, 0.0));


        std::string scan_topic = this->get_parameter("scan_topic").as_string();
        std::string scan_filtered_topic = this->get_parameter("scan_filtered_topic").as_string();
        std::string scan_nav_topic = this->get_parameter("scan_nav_topic").as_string();

        load_regions();

        scan_fitlered_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(scan_filtered_topic, rclcpp::QoS(rclcpp::KeepLast(10)));
        scan_nav_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(scan_nav_topic, rclcpp::QoS(rclcpp::KeepLast(10)));

        sub_scan_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            scan_topic, rclcpp::QoS(rclcpp::KeepLast(10)),
            std::bind(&AllFilterNode::scan_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Nodo inicializado" );
        print_configuration();
    }

private:
    void print_configuration() const {
        std::string input_topic = this->get_parameter("scan_topic").as_string();
        std::string out_topic_1 = this->get_parameter("scan_filtered_topic").as_string();
        std::string out_topic_2 = this->get_parameter("scan_nav_topic").as_string();

        RCLCPP_INFO(this->get_logger(), "========== CONFIGURACIÓN DEL NODO ==========");
        RCLCPP_INFO(this->get_logger(), "Topics:");
        RCLCPP_INFO(this->get_logger(), "  [Suscriptor] Entrada: '%s'", input_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "  [Publicador] Filtro sin autocolisiones : '%s'", out_topic_1.c_str());
        RCLCPP_INFO(this->get_logger(), "  [Publicador] Filtro para navegación  : '%s'", out_topic_2.c_str());
        
        RCLCPP_INFO(this->get_logger(), "Rangos Prohibidos [min_x, max_x, min_y, max_y]:");
        RCLCPP_INFO(this->get_logger(), "  -> Región navegación : [%.2f, %.2f, %.2f, %.2f]", 
                    region_nav_.min_x, region_nav_.max_x, region_nav_.min_y, region_nav_.max_y);

        for (size_t i = 0; i < 4; ++i) {
            RCLCPP_INFO(this->get_logger(), "  -> Región de autocolisión %zu de 4: [%.2f, %.2f, %.2f, %.2f]", 
                        i + 1, regions_filtered_[i].min_x, regions_filtered_[i].max_x, regions_filtered_[i].min_y, regions_filtered_[i].max_y);
        }
        RCLCPP_INFO(this->get_logger(), "============================================");
    }

    void load_regions() {
        auto region_nav_params = this->get_parameter("region_nav").as_double_array();
        if (region_nav_params.size() == 4) {
            region_nav_ = {static_cast<float>(region_nav_params[0]), static_cast<float>(region_nav_params[1]),
                         static_cast<float>(region_nav_params[2]), static_cast<float>(region_nav_params[3])};
        } else {
            RCLCPP_ERROR(this->get_logger(), "region_nav debe tener exactamente 4 elementos [min_x, max_x, min_y, max_y].");
        }

        auto regions_filtered_params = this->get_parameter("regions_filtered").as_double_array();
        if (regions_filtered_params.size() == 16) {
            for (size_t i = 0; i < 4; ++i) {
                regions_filtered_[i] = {static_cast<float>(regions_filtered_params[i*4 + 0]), static_cast<float>(regions_filtered_params[i*4 + 1]),
                                 static_cast<float>(regions_filtered_params[i*4 + 2]), static_cast<float>(regions_filtered_params[i*4 + 3])};
            }
        } else {
            RCLCPP_ERROR(this->get_logger(), "regions_filtered debe tener exactamente 16 elementos (4 regiones [min_x, max_x, min_y, max_y]).");
        }
    }

    void update_trig_cache(const sensor_msgs::msg::LaserScan::SharedPtr& msg) {
        // recalcular si cambia
        if (cached_angle_min_ != msg->angle_min ||
            cached_angle_inc_ != msg->angle_increment ||
            cached_size_ != msg->ranges.size()) 
        {
            cos_cache_.resize(msg->ranges.size());
            sin_cache_.resize(msg->ranges.size());
            
            for (size_t i = 0; i < msg->ranges.size(); ++i) {
                float angle = msg->angle_min + i * msg->angle_increment;
                cos_cache_[i] = std::cos(angle);
                sin_cache_[i] = std::sin(angle);
            }

            cached_angle_min_ = msg->angle_min;
            cached_angle_inc_ = msg->angle_increment;
            cached_size_ = msg->ranges.size();
            RCLCPP_INFO(this->get_logger(), "Caché trigonométrica actualizada para %zu rayos.", cached_size_);
        }
    }

    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        update_trig_cache(msg);

        // Crear copias del mensaje original para ambos outputs
        auto msg_out_4 = *msg;
        auto msg_out_1 = *msg;

        for (size_t i = 0; i < msg->ranges.size(); ++i) {
            float r = msg->ranges[i];

            // Ignorar lecturas no válidas 
            if (std::isinf(r) || std::isnan(r) || r < msg->range_min || r > msg->range_max) {
                continue; 
            }

            // Convertir polar a cartesiano usando caché 
            float x = r * cos_cache_[i];
            float y = r * sin_cache_[i];

            // Filtrado 1: Comprobar las 4 regiones de autocolision
            for (const auto& reg : regions_filtered_) {
                if (reg.contains(x, y)) {
                    msg_out_4.ranges[i] = msg->range_max; 
                    break;                     
                }
            }

            // Filtrado 2: Comprobar la región navegación
            if (region_nav_.contains(x, y)) {
                msg_out_1.ranges[i] = msg->range_max;
            }
        }

        // Publicar resultados
        scan_fitlered_pub_->publish(msg_out_4);
        scan_nav_pub_->publish(msg_out_1);
    }

    // Publicadores y Suscriptor
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_scan_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_fitlered_pub_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_nav_pub_;

    // Variables de las regiones
    RectRegion region_nav_;
    RectRegion regions_filtered_[4];

    // Caché para optimizar de conversión (Polar -> Cartesiano)
    std::vector<float> cos_cache_;
    std::vector<float> sin_cache_;
    float cached_angle_min_ = -1000.0f;
    float cached_angle_inc_ = -1000.0f;
    size_t cached_size_ = 0;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AllFilterNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}