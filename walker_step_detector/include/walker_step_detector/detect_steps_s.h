#ifndef DETECTSTEPSS_HH
#define DETECTSTEPSS_HH



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

// Local Headers
#include "walker_step_detector/legs_tracker.h"
#include "walker_step_detector/color_tools.h"

// Custom Messages related Headers
#include "slg_msgs/msg/segment_array.hpp"
#include "walker_msgs/msg/step_stamped.hpp"
#include "slg_msgs/segment2D.hpp"


// general cpp stuff
#include <fstream>
#include <memory>

using namespace std::chrono_literals;


class DetectStepsS : public rclcpp::Node
{
public:
    DetectStepsS();

private:
    

    
    bool is_debug;

    rclcpp::Time latest_scan_header_stamp_with_tf_available_;


    double marker_display_lifetime_;
    
    // configuration parameters
    std::string segments_topic_;
    std::string detected_steps_topic_name_;
    std::string detected_steps_frame_;
    double kalman_model_d0_, kalman_model_a0_, kalman_model_f0_, kalman_model_p0_;
    bool plot_leg_kalman_;
    bool plot_leg_clusters_;
    bool use_segment_header_stamp_for_tfs_;
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
    rclcpp::Subscription<slg_msgs::msg::SegmentArray>::SharedPtr segments_sub_;

    //..........................................................................
    std::list<walker_msgs::msg::StepStamped> getCentroids(std::string  fixed_frame_id, slg_msgs::msg::SegmentArray::SharedPtr segments_msg, std::shared_ptr<tf2_ros::Buffer> tf_buff);
    void segmentsCallback(const slg_msgs::msg::SegmentArray::SharedPtr segments_array);

    visualization_msgs::msg::Marker get_marker(const walker_msgs::msg::StepStamped* step, int id );

    void delete_markers();

    void publish_leg(walker_msgs::msg::StepStamped step, int sid);

};

#endif //DETECTSTEPSS_HH
