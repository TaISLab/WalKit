#include "stability_grid_map/stability_grid_map.hpp"

namespace stability_grid_map {

StabilityGridMap::StabilityGridMap()
: Node("stability_grid_map")
{
  if (!readParameters()) {
    return;
  }

  // Configure maps
  maps_.setFrameId(mapsFrameID_);
  maps_.setGeometry(grid_map::Length(mapsSizeX_, mapsSizeY_), mapsResolution_, grid_map::Position(mapsOriginX_, mapsOriginY_));
  nCols_ = maps_.getSize()(0);
  nRows_ = maps_.getSize()(1);
  
  // initial uniform prob is 1/numCells, so log prob is...
  initLogProb_ = - std::log(nCols_) - std::log(nRows_);

  //TODO: create timer callbacks: map rebuild
  //TODO: create timer callbacks: map publish

  // transform buffer
  buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tfl_ = std::make_shared<tf2_ros::TransformListener>(*buffer_);
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
                                                      this->get_node_base_interface(),
                                                      this->get_node_timers_interface());
  buffer_->setCreateTimerInterface(timer_interface);


  publisher_ = this->create_publisher<grid_map_msgs::msg::GridMap>(
    outputTopic_,
    rclcpp::QoS(1).transient_local());

  subscriber_ = this->create_subscription<walker_msgs::msg::StabilityStamped>(
    inputTopic_, 1,
    std::bind(&StabilityGridMap::callback, this, std::placeholders::_1));
}

StabilityGridMap::~StabilityGridMap(){
}

bool StabilityGridMap::readParameters(){
  // declare params ............
  this->declare_parameter<std::string>("input_topic",std::string("input"));
  this->declare_parameter<std::string>("output_topic", std::string("output"));
  this->declare_parameter<bool>("is_verbose", true );
  this->declare_parameter<std::string>("maps_frame_id", std::string("map"));
  this->declare_parameter<double>("maps_size_x",      12.0);
  this->declare_parameter<double>("maps_size_y",      10.0);
  this->declare_parameter<double>("maps_resolution",  0.1);
  this->declare_parameter<double>("maps_origin_x",    6.0);
  this->declare_parameter<double>("maps_origin_y",    5.0);
  this->declare_parameter<double>("maps_origin_y",    5.0);
  this->declare_parameter<double>("update_radius",    1.2);

  // read params ............
  this->get_parameter("input_topic", inputTopic_);
  this->get_parameter("output_topic", outputTopic_);
  this->get_parameter("is_verbose", isVerbose_);
  this->get_parameter("maps_frame_id", mapsFrameID_);
  this->get_parameter("maps_size_x", mapsSizeX_);
  this->get_parameter("maps_size_y", mapsSizeY_);
  this->get_parameter("maps_resolution", mapsResolution_);
  this->get_parameter("maps_origin_x", mapsOriginX_);
  this->get_parameter("maps_origin_y", mapsOriginY_);
  // TODO: how far do we update?
  this->get_parameter("update_radius", updateRadius_);


  // print params ............
  if (isVerbose_) {
    RCLCPP_INFO(this->get_logger(), "input_topic: [%s]", inputTopic_.c_str());
    RCLCPP_INFO(this->get_logger(), "output_topic: [%s]", outputTopic_.c_str());
    RCLCPP_INFO(this->get_logger(), "maps_frame_id: [%s]", mapsFrameID_.c_str());
    RCLCPP_INFO(this->get_logger(), "maps_size_x: [%3.3f]", mapsSizeX_);
    RCLCPP_INFO(this->get_logger(), "maps_size_y: [%3.3f]", mapsSizeY_);
    RCLCPP_INFO(this->get_logger(), "maps_resolution: [%3.3f]", mapsResolution_);
    RCLCPP_INFO(this->get_logger(), "maps_origin_x: [%3.3f]", mapsOriginX_);
    RCLCPP_INFO(this->get_logger(), "maps_origin_y: [%3.3f]", mapsOriginY_);
    RCLCPP_INFO(this->get_logger(), "update_radius: [%3.3f]", updateRadius_);
  }

  return true;
}

//TODO: timer callbacks: map rebuild
void StabilityGridMap::map_fusion_callback(){

// TODO: for all users in system
// TODO: read parameters and map
// TODO: merge down

}

void StabilityGridMap::map_publishg_callback(){

  if (isVerbose_){
    RCLCPP_INFO(this->get_logger(), "publishing merged map");
  }

  std::unique_ptr<grid_map_msgs::msg::GridMap> outputMessage;

  outputMessage = grid_map::GridMapRosConverter::toMessage(maps_);
  publisher_->publish(std::move(outputMessage));
}

void StabilityGridMap::merge_msg(const walker_msgs::msg::StabilityStamped::SharedPtr stab_msg){
  geometry_msgs::msg::PoseStamped globalPose, localPose;
  grid_map::Position center, point;
  double cellValue, radius;
  grid_map::Index ind;

  /*
     data in stab_msg
  // uid: user identificator
  // tin: tinetti score. The bigger the better shape the user is
  // sta: stability measurement, like risk value to be added.
  // pose: ros2 PoseStamped where measurement took place.
  */

  globalPose = stab_msg->pose;

  //Cast globalPose into map frame and get local center
  // wait 1 sec until give up transform...
  try {
      localPose = buffer_->transform<geometry_msgs::msg::PoseStamped>(globalPose, mapsFrameID_, tf2::Duration(std::chrono::seconds(1)));
  } catch (tf2::TransformException &e){
      RCLCPP_ERROR (this->get_logger(), "Cant transform from [%s] to [%s]. Reason [%s]", globalPose.header.frame_id.c_str(), mapsFrameID_.c_str(), e.what());
      return;
  }    

  // Use gridmap object to build circle iterator
  center.x() = localPose.pose.position.x;
  center.y() = localPose.pose.position.y;

  boost::math::normal_distribution<double> nd(0, 1/stab_msg->tin);

  // we will update a circle around event position center.
  // TODO: ANY SHAPE IS POSSIBLE ...
  for (grid_map::CircleIterator iterator(maps_, center, updateRadius_); !iterator.isPastEnd(); ++iterator) {

    ind = *iterator;
    // get cell center of current cell in the map frame.            
    maps_.getPosition(ind, point);

    // distance to iteration center:
    radius = std::sqrt( std::pow(center.x()-point.x(), 2.0) + std::pow(center.y()-point.y(), 2.0) );

    // prior value
    // maps_.at(stab_msg->uid, *iterator);

    /*TODO: magic goes here. We have:
      - radius         distance to event 
      - stab_msg->sta  event value
      - stab_msg->tin  user "confidence"?

    */ 
  
    cellValue = stab_msg->sta * boost::math::pdf(nd, radius);

    // update value
    maps_.at(stab_msg->uid, *iterator) += cellValue;
  }


}

void StabilityGridMap::callback(const walker_msgs::msg::StabilityStamped::SharedPtr stab_msg){

  // add msg to the log
  msgLog_.push_back(*stab_msg);

  // create the the layer if does not exitst
  if (!maps_.exists(stab_msg->uid)) {   
    // we use an uniform log(Prob) 
    maps_.add(stab_msg->uid, initLogProb_); 
  }

  // TODO merge reading with layer
  merge_msg(stab_msg);

}

}  // namespace grid_map_demos