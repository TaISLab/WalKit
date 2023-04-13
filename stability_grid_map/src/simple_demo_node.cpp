#include <rclcpp/rclcpp.hpp>
#include <memory>
#include "stability_grid_map/stability_grid_map.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<stability_grid_map::StabilityGridMap>());
  rclcpp::shutdown();
  return 0;
}