/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
* 
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
* 
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

// ROS related Headers
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_interface.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/transform_datatypes.h> 
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

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
    DetectSteps() : Node("detect_steps"){

        //srand (static_cast <unsigned> (time(0)));

        //Get ROS parameters
        std::string scan_topic;
        num_prev_markers_published_ = 0;
        scan_num_ = 0;
        double d0,a0,f0,p0;

        this->declare_parameter<std::string>("scan_topic",                "/scan");
        this->declare_parameter<std::string>("fixed_frame",               "laser");
        this->declare_parameter<std::string>("detect_distance_frame_id",  "base_link");
        this->declare_parameter<std::string>("forest_file",               "./src/leg_detector/config/trained_leg_detector_res_0.33.yaml");
        this->declare_parameter<std::string>("detected_steps_topic_name", "/detected_step");
        this->declare_parameter<double>("kalman_model_d0",                0.001);
        this->declare_parameter<double>("kalman_model_a0",                0.001);
        this->declare_parameter<double>("kalman_model_f0",                0.001);
        this->declare_parameter<double>("kalman_model_p0",                0.001);
        this->declare_parameter<double>("detection_threshold",            -1.0);
        this->declare_parameter<double>("cluster_dist_euclid",            0.13);
        this->declare_parameter<double>("max_detect_distance",            10.0);
        this->declare_parameter<double>("max_detected_clusters",          -1);
        this->declare_parameter<int>("min_points_per_cluster",            3);    
        this->declare_parameter<bool>("plot_all_clusters",                false);
        this->declare_parameter<bool>("plot_leg_kalman",                  false);
        this->declare_parameter<bool>("plot_leg_clusters",                false);
        this->declare_parameter<bool>("use_scan_header_stamp_for_tfs",    false);

        this->get_parameter("scan_topic",                    scan_topic);
        this->get_parameter("fixed_frame",                   fixed_frame_);
        this->get_parameter("detect_distance_frame_id",      detect_distance_frame_id_);
        this->get_parameter("forest_file",                   forest_file);
        this->get_parameter("detected_steps_topic_name",     detected_steps_topic_name_);
        this->get_parameter("kalman_model_d0",               d0);
        this->get_parameter("kalman_model_a0",               a0);
        this->get_parameter("kalman_model_f0",               f0);
        this->get_parameter("kalman_model_p0",               p0);
        this->get_parameter("detection_threshold",           detection_threshold_);
        this->get_parameter("cluster_dist_euclid",           cluster_dist_euclid_);
        this->get_parameter("max_detect_distance",           max_detect_distance_);
        this->get_parameter("max_detected_clusters",         max_detected_clusters_);
        this->get_parameter("min_points_per_cluster",        min_points_per_cluster_);
        this->get_parameter("plot_all_clusters",             plot_all_clusters_);
        this->get_parameter("plot_leg_kalman",               plot_leg_kalman_);
        this->get_parameter("plot_leg_clusters",             plot_leg_clusters_);
        this->get_parameter("use_scan_header_stamp_for_tfs", use_scan_header_stamp_for_tfs_);

        // Load kalman tracker
        kalman_tracker.init(this, d0, a0, f0, p0 );

        is_debug = (plot_all_clusters_ || plot_leg_clusters_|| plot_leg_kalman_);

        if (is_debug){
            kalman_tracker.enable_log();

            auto ret = rcutils_logging_set_logger_level( this->get_logger().get_name(), RCUTILS_LOG_SEVERITY_DEBUG);
            if (ret != RCUTILS_RET_OK) {
                RCLCPP_ERROR(this->get_logger(), "Error setting severity: %s", rcutils_get_error_string().str);
                rcutils_reset_error();
            }

            RCLCPP_INFO(this->get_logger(), "kalman model initial d: %.2f", d0);
            RCLCPP_INFO(this->get_logger(), "kalman model initial a: %.2f", a0);
            RCLCPP_INFO(this->get_logger(), "kalman model initial f: %.2f", f0);
            RCLCPP_INFO(this->get_logger(), "kalman model initial p: %.2f", p0);
            //Print the ROS parameters
            RCLCPP_INFO(this->get_logger(), "forest_file: %s", forest_file.c_str());
            RCLCPP_INFO(this->get_logger(), "scan_topic: %s", scan_topic.c_str());
            RCLCPP_INFO(this->get_logger(), "fixed_frame: %s", fixed_frame_.c_str());
            RCLCPP_INFO(this->get_logger(), "detection_threshold: %.2f", detection_threshold_);
            RCLCPP_INFO(this->get_logger(), "cluster_dist_euclid: %.2f", cluster_dist_euclid_);
            RCLCPP_INFO(this->get_logger(), "min_points_per_cluster: %d", min_points_per_cluster_);
            RCLCPP_INFO(this->get_logger(), "detect_distance_frame_id: %s", detect_distance_frame_id_.c_str());
            RCLCPP_INFO(this->get_logger(), "max_detect_distance: %.2f", max_detect_distance_);
            RCLCPP_INFO(this->get_logger(), "use_scan_header_stamp_for_tfs: %d", use_scan_header_stamp_for_tfs_);
            RCLCPP_INFO(this->get_logger(), "max_detected_clusters: %d", max_detected_clusters_);
        } else {
            RCLCPP_INFO(this->get_logger(), "Step detector loading. Set plot vars to true for debug.");
        }

        //Load Random forest
        processor.setForestFile(forest_file);

        latest_scan_header_stamp_with_tf_available_ = this->now();
        auto default_qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());

        left_detected_step_pub_ = this->create_publisher<walker_msgs::msg::StepStamped>(detected_steps_topic_name_ + "_left", 20);
        right_detected_step_pub_ = this->create_publisher<walker_msgs::msg::StepStamped>(detected_steps_topic_name_ + "_right", 20);

        if (is_debug){
            markers_array_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("clusters", 20);
        }

        this->scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(scan_topic, default_qos, std::bind(&DetectSteps::laserCallback, this, std::placeholders::_1));

        buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tfl_ = std::make_shared<tf2_ros::TransformListener>(*buffer_);
        auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
                                                            this->get_node_base_interface(),
                                                            this->get_node_timers_interface());
        buffer_->setCreateTimerInterface(timer_interface);
    }

private:

    std::shared_ptr<tf2_ros::Buffer> buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tfl_;
    
    std::string forest_file;
            
    
    int scan_num_;
    int num_prev_markers_published_;
    bool use_scan_header_stamp_for_tfs_;
    bool plot_all_clusters_;
    bool plot_leg_clusters_;
    bool plot_leg_kalman_;
    bool is_debug;
    rclcpp::Time latest_scan_header_stamp_with_tf_available_;

    std::string fixed_frame_;

    double detection_threshold_;
    double cluster_dist_euclid_;
    int min_points_per_cluster_;
    std::string detect_distance_frame_id_;
    double max_detect_distance_;
    double marker_display_lifetime_;
    int max_detected_clusters_;

    //create the publisher and subscribers
    std::string  detected_steps_topic_name_;
    rclcpp::Publisher<walker_msgs::msg::StepStamped>::SharedPtr left_detected_step_pub_;
    rclcpp::Publisher<walker_msgs::msg::StepStamped>::SharedPtr right_detected_step_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    //rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_array_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Tracker assigns scan clusters to legs and keeps track of them
    LegsTracker kalman_tracker;
    
    laser_processor::ScanProcessor processor;

    visualization_msgs::msg::Marker get_center_marker(const laser_processor::SampleSet* cluster, int id ){       
        std::string frame_id = "laser";
        std::string ns = "cluster";
        
        double alph = 0.9;

        double r = 1.0;
        double g = 0;
        double b = 0;
        double size = 0.05;

        visualization_msgs::msg::Marker marker;

        marker.header.frame_id = frame_id;
        marker.header.stamp = rclcpp::Clock().now();
        marker.lifetime = rclcpp::Duration(std::chrono::nanoseconds(0));

        marker.ns = ns;
        marker.id = id;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.type = visualization_msgs::msg::Marker::SPHERE;

        marker.pose.position = cluster->getPosition();            
        marker.pose.orientation.w = 1;

        // SPHERE markers use scale for diameter in each axis
        marker.scale.x = size;
        marker.scale.y = size;
        marker.scale.z = size;

        marker.color.r = r;
        marker.color.g = g;
        marker.color.b = b;
        marker.color.a = alph;

        return marker;
    }

    visualization_msgs::msg::Marker get_marker(const laser_processor::SampleSet* cluster, int id ){       
        std::string frame_id = "laser";
        std::string ns = "cluster";
        
        double alph = 0.8;
        RGB rgb0 = pick_one(id); 

        double r = rgb0.r; 
        double g = rgb0.g; 
        double b = rgb0.b; 
        double width = 0.02;
        double height = 0.02;
        double laser_z = 0.0;

        visualization_msgs::msg::Marker marker;

        marker.header.frame_id = frame_id;
        marker.header.stamp = rclcpp::Clock().now();
        marker.lifetime = rclcpp::Duration(std::chrono::nanoseconds(0));

        marker.ns = ns;
        marker.id = id;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.type = visualization_msgs::msg::Marker::POINTS;

        marker.pose.position.x = 0;
        marker.pose.position.y = 0;
        marker.pose.position.z = 0;        
        marker.pose.orientation.w = 1;

        // POINTS markers use x and y scale for width/height respectively
        marker.scale.x = width;
        marker.scale.y = height;

        marker.color.r = r;
        marker.color.g = g;
        marker.color.b = b;
        marker.color.a = alph;

        // Create the vertices for the points 
        for (auto sample : *cluster){
            geometry_msgs::msg::Point p;
            p.x = sample->x;
            p.y = sample->y;
            p.z = laser_z;
            marker.points.push_back(p);
        }

        return marker;
    }

    visualization_msgs::msg::Marker get_marker(const walker_msgs::msg::StepStamped* step, int id ){       
        std::string ns = "steps";
        
        RGB rgb0 = pick_one(id); 

        double r = rgb0.r; 
        double g = rgb0.g; 
        double b = rgb0.b;

        double size = 0.05;

        visualization_msgs::msg::Marker marker;

        marker.header = step->position.header;
        marker.lifetime = rclcpp::Duration(std::chrono::nanoseconds(0));

        marker.ns = ns;
        marker.id = id;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.type = visualization_msgs::msg::Marker::CUBE;

        marker.pose.position.x = step->position.point.x;
        marker.pose.position.y = step->position.point.y;
        marker.pose.position.z = step->position.point.z;        
        marker.pose.orientation.w = 1;

        marker.scale.x = size;
        marker.scale.y = size;
        marker.scale.z = size;        

        marker.color.r = r;
        marker.color.g = g;
        marker.color.b = b;
        marker.color.a = 1;

        return marker;
    }

    void delete_markers(){        
        visualization_msgs::msg::Marker marker_cluster;
        visualization_msgs::msg::MarkerArray marker_array;

        marker_cluster.action = visualization_msgs::msg::Marker::DELETEALL;
        marker_array.markers.push_back(marker_cluster);

        markers_array_pub_->publish(marker_array);
}

    void publish_clusters(std::list<laser_processor::SampleSet*> clusters,bool isYellow=false){
        int id = 0;
        for (auto cluster: clusters){
            publish_cluster(cluster, id, isYellow);            
            id = id + 2;
        }
    }

    void publish_cluster(laser_processor::SampleSet* cluster, int sid, bool isYellow=false){
        visualization_msgs::msg::MarkerArray marker_array;
        visualization_msgs::msg::Marker mark_i = get_marker(cluster, sid);
        marker_array.markers.push_back(mark_i);
        visualization_msgs::msg::Marker center_mark_i = get_center_marker(cluster, sid + 1);
        if (isYellow){
            center_mark_i.color.r = 1;
            center_mark_i.color.g = 1;
            center_mark_i.color.b = 0;
            center_mark_i.color.a = 1;
        }
        marker_array.markers.push_back(center_mark_i);
        markers_array_pub_->publish(marker_array);
    }

    void publish_leg(walker_msgs::msg::StepStamped step, int sid){
        visualization_msgs::msg::MarkerArray marker_array;
        visualization_msgs::msg::Marker mark_i = get_marker(&step, sid);

        mark_i.color.r = (sid==1 ? 1.0 : 0.0); 
        mark_i.color.g = 0; 
        mark_i.color.b = (sid==0 ? 1.0 : 0.0); 

        marker_array.markers.push_back(mark_i);
        markers_array_pub_->publish(marker_array);
    }


    /**
     * @brief Clusters the scan according to euclidian distance, 
     *        predicts the confidence that each cluster is a human leg and publishes the results
     * 
     * Called every time a laser scan is published.
     */
    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
    {
        walker_msgs::msg::StepStamped left_detected_step;
        walker_msgs::msg::StepStamped right_detected_step;

        processor.setScan(*scan);
        processor.splitConnected(cluster_dist_euclid_);

        if (is_debug){
            RCLCPP_DEBUG(this->get_logger(), "%d clusters with distances bigger than %3.3f", processor.size(), cluster_dist_euclid_); 
        }

        processor.removeLessThan(min_points_per_cluster_);

        if (is_debug){
            RCLCPP_DEBUG(this->get_logger(), "%d clusters with at least %d points", processor.size(), min_points_per_cluster_); 
        }

        // Find out the time that should be used for tfs
        bool transform_available;
        rclcpp::Time tf_time;
        geometry_msgs::msg::PointStamped position;

        // Use time from scan header
        if (use_scan_header_stamp_for_tfs_){
            tf_time = scan->header.stamp;
            try {
                buffer_->lookupTransform(fixed_frame_, scan->header.frame_id, tf_time, rclcpp::Duration(std::chrono::seconds(1)));
                transform_available = buffer_->canTransform(fixed_frame_, scan->header.frame_id, tf_time);              
            } catch(tf2::TransformException &e) {
                RCLCPP_WARN(this->get_logger(), "No tf available");
                transform_available = false;
                
            }
        } else {
            // Otherwise just use the latest tf available
            transform_available = buffer_->canTransform(fixed_frame_, scan->header.frame_id, tf_time);
        }

        if(!transform_available) {
            RCLCPP_WARN(this->get_logger(), "No TF available: no step detections can be done");
        } else {     

            // clear rviz 
            if (is_debug){
                delete_markers();
            }

            // view all clusters
            if (plot_all_clusters_){
                publish_clusters(processor.getClusters());
            }
       
            // Remove far clusters
            processor.removeFar(detect_distance_frame_id_, max_detect_distance_, scan->header, buffer_);

            if (is_debug){
                RCLCPP_DEBUG(this->get_logger(), "%d clusters closer than %3.3f to %s", processor.size(), max_detect_distance_, detect_distance_frame_id_.c_str()); 
            }

            // selected clusters for legs
            if (plot_leg_clusters_){
                publish_clusters(processor.getClusters(), true);
            }

            std::list<walker_msgs::msg::StepStamped> points = processor.getCentroids(fixed_frame_, scan->header, buffer_);
            
            kalman_tracker.add_detections(points);
        }
    
        // get steps from Kalman set
        walker_msgs::msg::StepStamped step_r;
        walker_msgs::msg::StepStamped step_l;
        double t = (this->now()).nanoseconds();

        kalman_tracker.get_steps(&step_r, &step_l, t);

        // publish lets
        right_detected_step_pub_->publish(step_r);
        left_detected_step_pub_->publish(step_l);
        
        if (is_debug){
            publish_leg(step_r, 0);
            publish_leg(step_l, 1);
            RCLCPP_DEBUG(this->get_logger(), ".......\n\n"); 
        }
    }

};

int main(int argc, char **argv)
{

    rclcpp::init(argc, argv);
    auto node = std::make_shared<DetectSteps>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
