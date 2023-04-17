#ifndef KM_DETECTSTEPS_HH
#define KM_DETECTSTEPS_HH


// ROS related Headers
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_interface.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2/transform_datatypes.h> 
#include "rcutils/error_handling.h"

// ROS MSGS
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

// Local Headers
#include "walker_step_detector/legs_tracker.h"
#include "walker_step_detector/color_tools.h"

// Custom Messages related Headers
#include "walker_msgs/msg/step_stamped.hpp"


// general cpp stuff
#include <fstream>
#include <memory>
#include <tuple>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include <vector>

#include "opencv2/core.hpp"
#include <opencv2/imgproc.hpp>

using namespace Eigen;
using namespace std::chrono_literals;



class KMDetectSteps : public rclcpp::Node
{
public:
    KMDetectSteps();

private:
    

    
    bool is_debug;

    rclcpp::Time latest_scan_header_stamp_with_tf_available_;


    double marker_display_lifetime_;
    
    // configuration parameters
    std::string scan_topic_;
    std::string detected_steps_topic_name_;
    std::string detected_steps_frame_;
    bool kalman_enabled_;
    bool fit_ellipse_;
    double kalman_model_d0_, kalman_model_a0_, kalman_model_f0_, kalman_model_p0_;
    bool plot_leg_kalman_;
    bool plot_leg_clusters_;
    bool use_scan_header_stamp_for_tfs_;
    std::vector<double> act_a_x_;
    std::vector<double> act_a_y_;

    // Tracker assigns segments to legs and keeps track of them
    LegsTracker kalman_tracker;

    //ROS objects
    rclcpp::TimerBase::SharedPtr timer_;

    // tf2 objects
    std::shared_ptr<tf2_ros::Buffer> buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tfl_;

    //publishers and subscribers
    rclcpp::Publisher<walker_msgs::msg::StepStamped>::SharedPtr left_detected_step_pub_;
    rclcpp::Publisher<walker_msgs::msg::StepStamped>::SharedPtr right_detected_step_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_array_pub_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr filter_laser_pub_ ;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;

    //..........................................................................
    std::list<walker_msgs::msg::StepStamped> getCentroids(sensor_msgs::msg::LaserScan::SharedPtr scan);
    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan);

    std::tuple<double, double> find_centroid(std::vector<double>& x, std::vector<double>& y, std::vector<unsigned int>& selected_indexs, std::vector<double>& selected_dists, double max_d);

    visualization_msgs::msg::Marker get_marker(const walker_msgs::msg::StepStamped* step, int id );

    void delete_markers();
    void fit_ellipse(double& result_center_x, double& result_center_y, 
                      double& result_phi, double& result_width, double& result_hight,
                      std::vector<double>& x,std::vector<double>& y, std::vector<unsigned int>& selected_indexs);

    void publish_leg(walker_msgs::msg::StepStamped step, int sid);
    void publish_active_area();
    double distance(double ax, double ay, double bx, double by);
};

#endif //KM_DETECTSTEPS_HH
