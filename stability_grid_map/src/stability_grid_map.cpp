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
  
  // layer containing all users data
  maps_.add( fusionLayerName_ , initMapVal_);
  
  // Internal state vars:
  newData_ = false;
  totalWeight_ = 0.0;

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

  
  //create timer callbacks: map rebuild
  mapFusionTimer_ = this->create_wall_timer(std::chrono::milliseconds(mapFusionTimerPeriodMilis_), std::bind(&StabilityGridMap::map_fusion_callback, this) );

  //create timer callbacks: map publish
  mapPublishTimer_ = this->create_wall_timer(std::chrono::milliseconds(mapPublishTimerPeriodMilis_), std::bind(&StabilityGridMap::map_publishg_callback, this) );
  if (isVerbose_) {
    RCLCPP_INFO(this->get_logger(), "Node started");
  }
}

StabilityGridMap::~StabilityGridMap(){
}

bool StabilityGridMap::readParameters(){
  // declare params ............
  this->declare_parameter<std::string>("stability_topic",std::string("user_stability"));
  this->declare_parameter<std::string>("stability_map_topic", std::string("stability_map"));
  this->declare_parameter<bool>("is_verbose", true );
  this->declare_parameter<std::string>("maps_frame_id", std::string("map"));
  this->declare_parameter<double>("maps_size_x",      12.0);
  this->declare_parameter<double>("maps_size_y",      10.0);
  this->declare_parameter<double>("maps_resolution",  0.1);
  this->declare_parameter<double>("maps_origin_x",    6.0);
  this->declare_parameter<double>("maps_origin_y",    5.0);
  // TODO: how far do we update?
  this->declare_parameter<double>("update_radius",    1.2);
  // TODO: how strong the fadding
  this->declare_parameter<double>("update_sigma",    1);
  this->declare_parameter<double>("initial_map_value",0.0);
  // TODO: how often do we update and publish?
  this->declare_parameter<int>("map_fusion_timer_ms", 500);
  this->declare_parameter<int>("map_publish_timer_ms", 1000); 

  // read params ............
  this->get_parameter("stability_topic", inputTopic_);
  this->get_parameter("stability_map_topic", outputTopic_);
  this->get_parameter("is_verbose", isVerbose_);
  this->get_parameter("maps_frame_id", mapsFrameID_);
  this->get_parameter("maps_size_x", mapsSizeX_);
  this->get_parameter("maps_size_y", mapsSizeY_);
  this->get_parameter("maps_resolution", mapsResolution_);
  this->get_parameter("maps_origin_x", mapsOriginX_);
  this->get_parameter("maps_origin_y", mapsOriginY_);  
  this->get_parameter("update_radius", updateRadius_);
  this->get_parameter("update_sigma", updateSigma_);
  this->get_parameter("initial_map_value", initMapVal_);
  this->get_parameter("map_fusion_timer_ms", mapFusionTimerPeriodMilis_);
  this->get_parameter("map_publish_timer_ms", mapPublishTimerPeriodMilis_);

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
    RCLCPP_INFO(this->get_logger(), "update_sigma: [%3.3f]", updateSigma_);
    RCLCPP_INFO(this->get_logger(), "initial_map_value: [%3.3f]", initMapVal_);
    RCLCPP_INFO(this->get_logger(), "map_fusion_timer_ms: [%d]", mapFusionTimerPeriodMilis_);
    RCLCPP_INFO(this->get_logger(), "map_publish_timer_ms: [%d]", mapPublishTimerPeriodMilis_);    
  }

  return true;
}

//global map rebuild
void StabilityGridMap::map_fusion_callback(){
  std::vector<std::string> layerNameList;

  layerNameList = maps_.getLayers();

  // We have one layer for the final fusion
  int numLayers = layerNameList.size()-1;

  if (numLayers>0){
    if (isVerbose_) {
      RCLCPP_INFO(this->get_logger(), "Fusing maps from [%d] users", numLayers);
      RCLCPP_INFO(this->get_logger(), "Total weight [%3.3f]",totalWeight_ );
    }

    // fusion map is the weighted overlapping of each users cumulative data.
    maps_.add(fusionLayerName_,0.0f);

    //TODO which weight do we add to each layer??
    //TODO do we even want to use a linear combination
    double weight;
      
    // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    // TODO:NO USAR GRIDMAPS: NEGRO INESTABLE Y BLANCO ESTABLE
    // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

      // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
      // TODO: GENERAR MAPAS USUARIOS 1 a 4 y GLOBAL
      // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!


    // iterate over layers in map collection
    for (const auto & layerName : layerNameList) {
      
      if (layerName != fusionLayerName_){
        weight = tinetti_dict_[layerName]/ totalWeight_;
        if (isVerbose_) {
          RCLCPP_INFO(this->get_logger(), "User [%s] weight [%3.3f]",stab_msg->uid.c_str(), weight );
        }
        // merge down
        maps_[fusionLayerName_] += weight * maps_[layerName] ;
      }
    }
    //once added: reescale average layer, so that every cell is between 0-1
    // const double minValue = maps_.get(fusionLayerName_).minCoeffOfFinites();
    const double maxValue = maps_.get(fusionLayerName_).maxCoeffOfFinites();
    
    maps_[fusionLayerName_] = maps_[fusionLayerName_]/maxValue;
  } 
}

void StabilityGridMap::map_publishg_callback(){
  std::unique_ptr<grid_map_msgs::msg::GridMap> outputMessage;

  if (newData_ || isVerbose_){
    outputMessage = grid_map::GridMapRosConverter::toMessage(maps_);
    outputMessage->header.stamp = this->now();
    publisher_->publish(std::move(outputMessage));

    if (isVerbose_){
      RCLCPP_INFO(this->get_logger(), "publishing merged map");
    }

    newData_ = false;
  }
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
  geometry_msgs::msg::TransformStamped transformStamped;

  // transform with "latest" transform available...
  try {               
    transformStamped = buffer_->lookupTransform(
        mapsFrameID_, globalPose.header.frame_id,
        tf2::TimePointZero, 
        tf2::Duration(std::chrono::milliseconds(50)));
  } catch (tf2::TransformException &e){
      RCLCPP_ERROR (this->get_logger(), "Cant transform from [%s] to [%s]. Reason [%s]", globalPose.header.frame_id.c_str(), mapsFrameID_.c_str(), e.what());
      return;
  }    
  
  tf2::doTransform(globalPose, localPose, transformStamped);

  //if (isVerbose_) {
  //  RCLCPP_INFO(this->get_logger(), "User is at   [%3.3f, %3.3f] [%s]", globalPose.pose.position.x, globalPose.pose.position.y, globalPose.header.frame_id.c_str() );
  //  RCLCPP_INFO(this->get_logger(), "In map frame [%3.3f, %3.3f] [%s]", localPose.pose.position.x,  localPose.pose.position.y,  localPose.header.frame_id.c_str() );
  //}

  // Use gridmap object to build circle iterator
  center.x() = localPose.pose.position.x;
  center.y() = localPose.pose.position.y;

  //boost::math::normal_distribution<double> nd(0, 1/stab_msg->tin);

  if (isVerbose_) {
    RCLCPP_INFO(this->get_logger(), "Update for user [%s] with TIN [%3.3f] Stability value [%3.3f]",stab_msg->uid.c_str(), stab_msg->tin, stab_msg->sta  );
  }
  // we will update a circle around event position center.
  // TODO: ANY SHAPE IS POSSIBLE ...
  for (grid_map::CircleIterator iterator(maps_, center, updateRadius_); !iterator.isPastEnd(); ++iterator) {

      // keep count
      maps_.at(stab_msg->uid + "_count", *iterator) += 1.0;

      ind = *iterator;
      // get cell center of current cell in the map frame.            
      maps_.getPosition(ind, point);

      // distance to iteration center:
      radius = std::sqrt( std::pow(center.x()-point.x(), 2.0) + std::pow(center.y()-point.y(), 2.0) );

      // prior value
      double prev_value = maps_.at(stab_msg->uid, *iterator);

      // number of readings plus one
      double n_readings =  maps_.at(stab_msg->uid + "_count", *iterator) ;
      /*TODO: magic goes here. We have:
        - radius         distance to event 
        - stab_msg->sta  event value
        - stab_msg->tin  user "confidence"?

      */ 
    
      
      double dispersion = normal_pdf(radius, 0, updateSigma_);
      cellValue = stab_msg->sta * dispersion;
      
      if (isVerbose_) {
        RCLCPP_INFO(this->get_logger(), "Cell[%3.3f, %3.3f| %3.3f] = [%3.3f]", point.x(), point.y(),radius, maps_.at(stab_msg->uid, *iterator) );
        RCLCPP_INFO(this->get_logger(), "sta *dispersion = [%3.3f] * [%3.3f]", cellValue, dispersion);
      }

      // update value
      maps_.at(stab_msg->uid, *iterator) = (prev_value*(n_readings-1.0) + cellValue ) / n_readings;

      if (isVerbose_) {
        RCLCPP_INFO(this->get_logger(), "Final val = [%3.3f]", maps_.at(stab_msg->uid, *iterator) );
      }

  }

  if (isVerbose_) {
    RCLCPP_INFO(this->get_logger(), "Stability msg from user [%s] merged",stab_msg->uid.c_str() );
  }

  newData_ = true;
}

/*
A gaussian with probability density given by:
    pdf = 1/(s*sqrt(2*pi))  * exp(- (x-m)^2/(2*s^2)  )
Will have a log probability as:
    ln_pdf = -ln(s) - 0.5 ln(2*pi)  - (x-m)^2/(2*s^2)  
*/

double StabilityGridMap::normal_log_pdf(double x, double m, double s){
    static const double half_ln_2pi = 0.918938533;
    double a    = (x - m) / s;
    double ln_s = std::log(s);
    
    double log_pdf = - ( ln_s + half_ln_2pi + (a * a * 0.5) );

    return log_pdf;
}

/*
Let x be a random var with 
  mean m 
  standard deviation s
Its probability will be:
    pdf = 1/(s*sqrt(2*pi))  * exp(- (x-m)^2/(2*s^2)
*/

double StabilityGridMap::normal_pdf(double x, double m, double s){
    static const double  inv_sqrt_2pi = 0.3989422804014327;
    double a    = (x - m) / s; 
    
    double pdf_x = inv_sqrt_2pi / s * std::exp(-0.5 * a * a);

    return pdf_x;
}

void StabilityGridMap::callback(const walker_msgs::msg::StabilityStamped::SharedPtr stab_msg){

  // add msg to the log
  msgLog_.push_back(*stab_msg);

  // create the the layer if does not exitst
  if (!maps_.exists(stab_msg->uid)) {   
    // store its tinetti value
    tinetti_dict_[stab_msg->uid] = stab_msg->tin;
    totalWeight_+= stab_msg->tin;
    // we use an uniform log(Prob) 
    maps_.add(stab_msg->uid, initMapVal_); 
    // keep count
    maps_.add(stab_msg->uid + "_count", 0); 
    if (isVerbose_) {
      RCLCPP_INFO(this->get_logger(), "First msg from user [%s]",stab_msg->uid.c_str() );
    }
  }

  // merge reading with its layer
  merge_msg(stab_msg);

}

}  // namespace grid_map_demos