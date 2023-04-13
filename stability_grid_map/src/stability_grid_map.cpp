#include "stability_grid_map/stability_grid_map.hpp"

namespace stability_grid_map {

StabilityGridMap::StabilityGridMap()
: Node("stability_grid_map")
{
  if (!readParameters()) {
    return;
  }

  subscriber_ = this->create_subscription<grid_map_msgs::msg::GridMap>(
    inputTopic_, 1,
    std::bind(&StabilityGridMap::callback, this, std::placeholders::_1));

  publisher_ = this->create_publisher<grid_map_msgs::msg::GridMap>(
    outputTopic_,
    rclcpp::QoS(1).transient_local());

}

StabilityGridMap::~StabilityGridMap()
{
}

bool StabilityGridMap::readParameters()
{
  this->declare_parameter<std::string>("input_topic");
  this->declare_parameter("output_topic", std::string("output"));

  if (!this->get_parameter("input_topic", inputTopic_)) {
    RCLCPP_ERROR(this->get_logger(), "Could not read parameter `input_topic`.");
    return false;
  }

  this->get_parameter("output_topic", outputTopic_);
  return true;
}

void StabilityGridMap::callback(const walker_msgs::msg::StabilityStamped::SharedPtr stab_msg)
{
  // Convert stab_msg to map.
  grid_map::GridMap inputMap;
  grid_map::GridMapRosConverter::fromMessage(*stab_msg, inputMap);

  // Apply filter chain.
  grid_map::GridMap outputMap;
  if (!filterChain_.update(inputMap, outputMap)) {
    RCLCPP_ERROR(this->get_logger(), "Could not update the grid map filter chain!");
    return;
  }

  RCLCPP_INFO(this->get_logger(), "PUBLISH");
  // Publish filtered output grid map.
  std::unique_ptr<grid_map_msgs::msg::GridMap> outputMessage;
  outputMessage = grid_map::GridMapRosConverter::toMessage(outputMap);
  publisher_->publish(std::move(outputMessage));
}

}  // namespace grid_map_demos