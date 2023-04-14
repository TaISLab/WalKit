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

  //TODO: create timer callbacks: map rebuild
  //TODO: create timer callbacks: map publish

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
  geometry_msgs::msg::PoseStamped globalPose;
  grid_map::Position center, point;
  double radius, cellValue, c_x, c_y;
  grid_map::Index ind;

  // data in stab_msg
  // user identificator
  // uid
  
  // tinetti: the bigger the better shape the user is
  // tin

  // stability: risk value to be added.
  // sta

  // where did it happened
  // PoseStamped pose

  globalPose = stab_msg->pose;

  //TODO: cast globalPose into map frame and get local center c_x, c_y
  c_x = 0;
  c_y = 0;
  cellValue=0;

  center.x()= c_x;
  center.y()= c_y;

  // TODO: how far do we update?
  radius = 1.2;  // roughly 3 walkers

  // we will update a circle around event position center.
  for (grid_map::CircleIterator iterator(maps_, center, radius); !iterator.isPastEnd(); ++iterator) {

    ind = *iterator;
    // get cell center of current cell in the map frame.            
    maps_.getPosition(ind, point);

    // distance to iteration center:
    radius = std::sqrt( std::pow(center.x()-point.x()) + std::pow(center.y()-point.y()) );

    // prior value
    cellValue = maps_.at(stab_msg->uid, *iterator);

    /*TODO: magic goes here. We have:
      - radius         distance to event 
      - stab_msg->sta  event value
      - stab_msg->tin  user "confidence"?
      - cellValue      previous value
    */ 

    // update value
    maps_.at(stab_msg->uid, *iterator) = cellValue;
  }


}

void StabilityGridMap::callback(const walker_msgs::msg::StabilityStamped::SharedPtr stab_msg){

  // add msg to the log
  msgLog_.push_back(*stab_msg);

  // create the the layer if does not exitst
  if (!maps_.exists(stab_msg->uid)) {    
    maps_.add(stab_msg->uid, 0); 
  }

  // TODO merge reading with layer
  merge_msg(stab_msg);

}

}  // namespace grid_map_demos