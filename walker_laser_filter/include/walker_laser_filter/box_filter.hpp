#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>


using namespace std::chrono_literals;

class BoxFilter : public rclcpp::Node
{
    public:
        ////////////////////////////////////////////////////////////////////////////////
        BoxFilter(rclcpp::NodeOptions options);

    private:
        std::string in_scan_topic_name_;
        std::string out_scan_topic_name_;
        std::string param_prefix_;

        rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr out_scan_pub_;
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr in_scan_sub_;
        rclcpp::node_interfaces::NodeParametersInterface::SharedPtr params_interface_;

        double max_x_;
        double max_y_;
        double min_x_;
        double min_y_;
        bool remove_out_;    
        ////////////////////////////////////////////////////////////////////////////////
        void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan);
        bool isValid(double x, double y);
        template<typename PT>
        bool initParamImpl(const std::string & name, const uint8_t type, PT default_value, PT & value_out);
        bool initParam(const std::string & name, bool value_default, bool & value_out);
        bool initParam(const std::string & name, std::string value_default, std::string & value_out);
        bool initParam(const std::string & name, double value_default, double & value_out);


} ;



