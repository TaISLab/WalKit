
#include "walker_diff_odom/walker_odom.hpp"

int main(int argc, char **argv)
{

	rclcpp::init(argc, argv);
	auto diff_node = std::make_shared<WalkerDiffDrive>();	
    rclcpp::spin(diff_node);

	rclcpp::shutdown();
	return 0;

}
