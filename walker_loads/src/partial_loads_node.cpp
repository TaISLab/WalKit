//#include "walker_loads/partial_loads.h"


#include "rcpputils/split.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char **argv){

	rclcpp::init(argc, argv);
	auto node = rclcpp::Node::make_shared("minimal_publisher");

/*
	rclcpp::init(argc, argv);
	auto node = std::make_shared<PartialLoads>();	
    RCLCPP_INFO(node->get_logger(), "Starting Partial Loads node. ");

	rclcpp::spin(node);
    RCLCPP_INFO(node->get_logger(), "Shutting down Partial Loads node. ");
	rclcpp::shutdown();
*/
std::string hola = "miquel:34:leñador";
auto tokens = rcpputils::split(hola, ':', true);
RCLCPP_INFO(node->get_logger(), "token len [%d]. ", tokens.size());
RCLCPP_INFO(node->get_logger(), "token 2 [%s]. ", tokens[1].c_str());

return 0;

}
