#ifndef DETECTSTEPS_HH
#define DETECTSTEPS_HH



// ROS related Headers
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_interface.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/transform_datatypes.h> 
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
//#include <sensor_msgs/msg/point_cloud2.hpp>
//#include <pcl_conversions/pcl_conversions.h>

#include "rcutils/error_handling.h"

// Local Headers
#include "walker_step_detector/laser_processor.h"
#include "walker_step_detector/legs_tracker.h"
#include "walker_step_detector/color_tools.h"

// Custom Messages related Headers
#include "walker_msgs/msg/step_stamped.hpp"

#include <fstream>
#include <memory>
//#include <ctime>
//#include <cstdlib>

using namespace std::chrono_literals;


class DetectSteps : public rclcpp::Node
{
public:
    DetectSteps();

private:
    

    bool publish_clusters_;
    bool is_debug;
    std::string scan_topic_name_;
    std::string forest_file;
    std::string detected_steps_topic_name_;
    double kalman_model_d0_;
    double kalman_model_a0_;
    double kalman_model_f0_;
    double kalman_model_p0_;

    double detection_threshold_;
    double cluster_dist_euclid_;
    int min_points_per_cluster_;
    double max_detect_distance_;
    double marker_display_lifetime_;
    int max_detected_clusters_;

    rclcpp::Publisher<walker_msgs::msg::StepStamped>::SharedPtr left_detected_step_pub_;
    rclcpp::Publisher<walker_msgs::msg::StepStamped>::SharedPtr right_detected_step_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_array_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr points_pub_;
    LegsTracker kalman_tracker;    
    laser_processor::ScanProcessor processor;

    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan);
    std::list<walker_msgs::msg::StepStamped> getCentroids(sensor_msgs::msg::LaserScan::SharedPtr scan);
};

#endif //DETECTSTEPS_HH
