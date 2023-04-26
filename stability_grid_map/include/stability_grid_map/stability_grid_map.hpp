
#ifndef GRID_MAP_DEMOS__STABILITYGRIDMAP_HPP_
#define GRID_MAP_DEMOS__STABILITYGRIDMAP_HPP_

#include <string>
#include <memory>
#include <utility>
#include <list>
#include <tuple>
#include <map>
#include <chrono>

//#include <boost/random.hpp>
//#include <boost/random/normal_distribution.hpp>
// #include <boost/math/distributions/normal.hpp> // for normal_distribution

#include <rclcpp/rclcpp.hpp>
#include <grid_map_ros/grid_map_ros.hpp>


#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_interface.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2/transform_datatypes.h> 

// ROS MSGS
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>


#include "walker_msgs/msg/stability_stamped.hpp"
 

namespace stability_grid_map
{

/*!
 * Handles user specific stability data.
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
   * Callback method for the incoming stability message.
   * @param stab_msg the incoming message.
   */
  void callback(const walker_msgs::msg::StabilityStamped::SharedPtr stab_msg);

  /*!
   * Timer Callback method calling for single user map fusion
   */
  void map_fusion_callback();

  /*!
   * Timer Callback method calling merged map publish
   */
  void map_publishg_callback();

  /*!
   * Merges a msg into its corresponding map layer
   */
  void merge_msg(const walker_msgs::msg::StabilityStamped::SharedPtr stab_msg);

private:
  const std::string fusionLayerName_ = "fusion";

  std::list<walker_msgs::msg::StabilityStamped> msgLog_;
  
  grid_map::GridMap maps_;

  //! Name of the input stability topic.
  std::string inputTopic_;

  //! Name of the output grid map topic.
  std::string outputTopic_;

  //! Be verbose ...
  bool isVerbose_;

  // tf2 objects
  std::shared_ptr<tf2_ros::Buffer> buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tfl_;

  std::string mapsFrameID_;
  double mapsSizeX_;
  double mapsSizeY_;
  double mapsResolution_;
  double mapsOriginX_;
  double mapsOriginY_;
  double updateRadius_;
  int mapFusionTimerPeriodMilis_;
  int mapPublishTimerPeriodMilis_;

  int nCols_ ;
  int nRows_ ;
  double initMapVal_;

  //! stability subscriber
  rclcpp::Subscription<walker_msgs::msg::StabilityStamped>::SharedPtr subscriber_;

  //! Grid map publisher.
  rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr publisher_;
  
  rclcpp::TimerBase::SharedPtr mapFusionTimer_;
  rclcpp::TimerBase::SharedPtr mapPublishTimer_;

double normal_log_pdf(double x, double m, double s);
double normal_pdf(double x, double m, double s);

};

}  // namespace grid_map_demos
#endif  // GRID_MAP_DEMOS__STABILITYGRIDMAP_HPP_