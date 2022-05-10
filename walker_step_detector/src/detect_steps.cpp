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
#include <memory>

// OpenCV related Headers
#include <opencv2/core/core.hpp>
#include <opencv2/ml/ml.hpp>
#include <opencv2/ml.hpp>
#include <opencv2/core/types_c.h>
#include <opencv2/core/core_c.h> 

// Local Headers
#include "walker_step_detector/cluster_features.h"
#include "walker_step_detector/laser_processor.h"
#include "walker_step_detector/legs_tracker.h"

// Custom Messages related Headers
#include "walker_msgs/msg/step_stamped.hpp"

#include <fstream>

class DetectSteps : public rclcpp::Node
{
public:
    DetectSteps() : Node("detect_steps")
    {

        //Get ROS parameters
        std::string forest_file;
        std::string scan_topic;
        num_prev_markers_published_ = 0;
        scan_num_ = 0;

        this->declare_parameter("scan_topic");
        this->declare_parameter("fixed_frame");
        this->declare_parameter("forest_file");
        this->declare_parameter("detection_threshold");
        this->declare_parameter("cluster_dist_euclid");
        this->declare_parameter("min_points_per_cluster");
        this->declare_parameter("detect_distance_frame_id");
        this->declare_parameter("max_detect_distance");
        this->declare_parameter("use_scan_header_stamp_for_tfs");
        this->declare_parameter("max_detected_clusters");
        this->declare_parameter("detected_steps_topic_name");

        this->get_parameter_or("scan_topic", scan_topic, std::string("/scan"));
        this->get_parameter_or("fixed_frame", fixed_frame_, std::string("laser"));
        this->get_parameter_or("forest_file", forest_file, std::string("./src/leg_detector/config/trained_leg_detector_res_0.33.yaml"));
        this->get_parameter_or("detection_threshold", detection_threshold_, -1.0);
        this->get_parameter_or("cluster_dist_euclid", cluster_dist_euclid_, 0.13);
        this->get_parameter_or("min_points_per_cluster", min_points_per_cluster_, 3);
        this->get_parameter_or("detect_distance_frame_id", detect_distance_frame_id_, std::string("base_link"));
        this->get_parameter_or("max_detect_distance", max_detect_distance_, 10.0);
        this->get_parameter_or("use_scan_header_stamp_for_tfs", use_scan_header_stamp_for_tfs_, false);
        this->get_parameter_or("max_detected_clusters", max_detected_clusters_, -1);
        this->get_parameter_or("detected_steps_topic_name", detected_steps_topic_name_, std::string("/detected_step"));


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

        //Load Random forest
        forest = cv::ml::StatModel::load<cv::ml::RTrees>(forest_file);
        feat_count_ = forest->getVarCount();

        latest_scan_header_stamp_with_tf_available_ = this->now();
        auto default_qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());

        left_detected_step_pub_ = this->create_publisher<walker_msgs::msg::StepStamped>(detected_steps_topic_name_ + "_left", 20);
        right_detected_step_pub_ = this->create_publisher<walker_msgs::msg::StepStamped>(detected_steps_topic_name_ + "_right", 20);
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
    
            
    cv::Ptr<cv::ml::RTrees> forest = cv::ml::RTrees::create();

    int feat_count_;

    ClusterFeatures cf_;
    
    int scan_num_;
    int num_prev_markers_published_;
    bool use_scan_header_stamp_for_tfs_;

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

    // Tracker assigns scan clusters to legs and keeps track of them
    LegsTracker kalman_tracker;

    /**
     * @brief Clusters the scan according to euclidian distance, 
     *        predicts the confidence that each cluster is a human leg and publishes the results
     * 
     * Called every time a laser scan is published.
     */
    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
    {
        laser_processor::ScanProcessor processor(*scan);
        processor.splitConnected(cluster_dist_euclid_);
        processor.removeLessThan(min_points_per_cluster_);
        
        // OpenCV matrix needed to use the OpenCV random forest classifier
        CvMat* tmp_mat = cvCreateMat(1, feat_count_, CV_32FC1);

        walker_msgs::msg::StepStamped left_detected_step;
        walker_msgs::msg::StepStamped right_detected_step;
/*
        left_detected_step.position.header.frame_id = scan->header.frame_id;
        left_detected_step.position.header.stamp = scan->header.stamp;
        right_detected_step.position.header = left_detected_step.position.header;
*/
        // Find out the time that should be used for tfs
        bool transform_available;
        rclcpp::Time tf_time;
        
        // Use time from scan header
        if (use_scan_header_stamp_for_tfs_) 
        {
            tf_time = scan->header.stamp;

            try {
                buffer_->lookupTransform(fixed_frame_, scan->header.frame_id, tf_time, rclcpp::Duration(1.0));
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
            RCLCPP_WARN(this->get_logger(), "Not publishing detected legs because no tf was available");
        } else {
            // Iterate through all clusters
            for (std::list<laser_processor::SampleSet*>::iterator cluster = processor.getClusters().begin();cluster != processor.getClusters().end(); cluster++) {
                // Cluster position in laser frame
                geometry_msgs::msg::PointStamped position;
                position.header = scan->header;
                position.point = (*cluster)->getPosition();

                // transform
                geometry_msgs::msg::PointStamped position_new;
                float rel_dist = 4092.0;
                try {
                    buffer_->transform(position, position_new, detect_distance_frame_id_);
                    rel_dist = pow(position_new.point.x*position_new.point.x + position_new.point.y*position_new.point.y, 1./2.);
                } catch (tf2::TransformException &e){
                    RCLCPP_ERROR (this->get_logger(), "%s", e.what());
                }

                // Only consider clusters within max_distance
                if (rel_dist < max_detect_distance_) {

                    // Classify cluster using random forest classifier
                    std::vector<float> f = cf_.calcClusterFeatures(*cluster, *scan);
                    for (int k = 0; k < feat_count_; k++)
                        tmp_mat->data.fl[k] = (float)(f[k]);
                    
                    #if (CV_VERSION_MAJOR <= 3 || CV_VERSION_MINOR <= 2)
                        // Output of forest->predict is [-1.0, 1.0] so we scale to reach [0.0, 1.0]
                        float probability_of_leg = 0.5 * (1.0 + forest->predict(cv::cvarrToMat(tmp_mat)));
                    #else
                        // The forest->predict function has been removed in the latest versions of OpenCV so we'll do the calculation explicitly.
                        RCLCPP_INFO (this->get_logger(), "Checkout 6");
                        cv::Mat result;
                        forest->getVotes(cv::cvarrToMat(tmp_mat), result, 0);
                        int positive_votes = result.at<int>(1, 1);
                        int negative_votes = result.at<int>(1, 0);
                        float probability_of_leg = positive_votes / static_cast<double>(positive_votes + negative_votes);
                    #endif

                    // Consider only clusters that have a confidence greater than detection_threshold_
                    if (probability_of_leg > detection_threshold_){
                        // Transform cluster position to fixed frame
                        // This should always be successful because we've checked earlier if a tf was available
                        bool transform_successful_2;
                        try {
                            buffer_->transform(position, position, fixed_frame_);
                            transform_successful_2 = true;
                        } catch (tf2::TransformException &e){
                            RCLCPP_ERROR (this->get_logger(), "%s", e.what());
                            transform_successful_2 = false;
                        }

                        if (transform_successful_2) {
                            // keep track of potential detections
                            kalman_tracker.add_detection(position, probability_of_leg);
                        }
                    }
                    
                }
            }
        }


        // get steps from Kalman set
        walker_msgs::msg::StepStamped step_r;
        walker_msgs::msg::StepStamped step_l;
        double t = (this->now()).nanoseconds();

        kalman_tracker.get_steps(&step_r, &step_l, t);

        // publish lets
        right_detected_step_pub_->publish(step_r);
        left_detected_step_pub_->publish(step_l);
        cvReleaseMat(&tmp_mat);
    }


    void print_step(walker_msgs::msg::StepStamped step, std::string text){
        
        RCLCPP_INFO(this->get_logger(), "Pose %s: %3.3f, %3.3f (%3.3f)", text.c_str(), step.position.point.x, step.position.point.y, step.confidence);

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
