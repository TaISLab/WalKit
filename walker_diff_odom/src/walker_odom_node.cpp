
#include "walker_diff_odom/walker_odom.hpp"

int main(int argc, char **argv)
{

	rclcpp::init(argc, argv);
	auto diff_node = std::make_shared<WalkerDiffDrive>();	
    RCLCPP_INFO(diff_node->get_logger(), "Starting odometry node. ");

	rclcpp::spin(diff_node);
    RCLCPP_INFO(diff_node->get_logger(), "Finishing odometry node. ");
	rclcpp::shutdown();
	return 0;

}
