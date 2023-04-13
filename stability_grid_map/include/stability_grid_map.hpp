
#ifndef GRID_MAP_DEMOS__STABILITYGRIDMAP_HPP_
#define GRID_MAP_DEMOS__STABILITYGRIDMAP_HPP_

#include <rclcpp/rclcpp.hpp>
#include <string>

namespace stability_grid_map
{

/*!
 * 
 */
class StabilityGridMap : public rclcpp::Node
{
public:
  /*!
   * Constructor.
   */
  StabilityGridMap();

  /*!
   * Destructor.
   */
  virtual ~StabilityGridMap();

  /*!
  * Reads and verifies the ROS parameters.
  * @return true if successful.
  */
  bool readParameters();

  /*!
   * Callback method for the incoming grid map message.
   * @param message the incoming message.
   */
  void callback(const walker_msgs::msg::StabilityStamped::SharedPtr stab_msg);

private:
  //! Name of the input grid map topic.
  std::string inputTopic_;

  //! Name of the output grid map topic.
  std::string outputTopic_;

  //! Grid map subscriber
  rclcpp::Subscription<grid_map_msgs::msg::GridMap>::SharedPtr subscriber_;

  //! Grid map publisher.
  rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr publisher_;
  
};

}  // namespace grid_map_demos
#endif  // GRID_MAP_DEMOS__STABILITYGRIDMAP_HPP_