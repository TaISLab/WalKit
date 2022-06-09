#include "walker_loads/partial_loads.h"

int main(int argc, char **argv){

	rclcpp::init(argc, argv);
	auto node = std::make_shared<PartialLoads>();	
    RCLCPP_INFO(node->get_logger(), "Starting Partial Loads node. ");

	rclcpp::spin(node);
    RCLCPP_INFO(node->get_logger(), "Shutting down Partial Loads node. ");
	rclcpp::shutdown();

	return 0;

}
