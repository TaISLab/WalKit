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
#include <visualization_msgs/msg/marker.hpp>

// Custom Messages related Headers
#include "walker_msgs/msg/step_stamped.hpp"

#include <fstream>

class PlotSteps : public rclcpp::Node{

public:
    PlotSteps() : Node("steps_plotter"){
        
        // declare parameters
        this->declare_parameter<std::string>("steps_topic_name", "/detected_step");
        this->declare_parameter<std::string>("markers_topic_name", "/steps_markers");
        this->declare_parameter<std::string>("markers_namespace", "steps");
        this->declare_parameter<double>("marker_display_lifetime", 0.01);
        this->declare_parameter<double>("speed_dead_zone", 0.05);
        this->declare_parameter<double>("marker_size", 0.05);
        this->declare_parameter<double>("min_confidence", 0.0);
        this->declare_parameter<bool>("pub_position", false);

        // default values
        this->get_parameter("steps_topic_name", steps_topic_name_);
        this->get_parameter("markers_topic_name", markers_topic_name_);
        this->get_parameter("markers_namespace", markers_namespace_);
        this->get_parameter("marker_display_lifetime", marker_display_lifetime_);  // seconds
        this->get_parameter("speed_dead_zone", speed_dead_zone_);
        this->get_parameter("marker_size", marker_size_);
        this->get_parameter("min_confidence", min_confidence_);
        this->get_parameter("pub_position", pub_position_);
        
        
        //Print  parameters
        RCLCPP_INFO(this->get_logger(), "Node parameters: ");
        RCLCPP_INFO(this->get_logger(), "steps_topic_name: %s", steps_topic_name_.c_str());
        RCLCPP_INFO(this->get_logger(), "markers_topic_name: %s", markers_topic_name_.c_str());
        RCLCPP_INFO(this->get_logger(), "markers_namespace: %s", markers_namespace_.c_str());        
        RCLCPP_INFO(this->get_logger(), "marker_display_lifetime: %.2f", marker_display_lifetime_);
        RCLCPP_INFO(this->get_logger(), "speed_dead_zone: %.2f", speed_dead_zone_);
        RCLCPP_INFO(this->get_logger(), "marker_size: %.2f", marker_size_);
        RCLCPP_INFO(this->get_logger(), "min_confidence: %.2f", min_confidence_);
        RCLCPP_INFO(this->get_logger(), "pub_position: %s", pub_position_ ? "True" : "False");

        // pub/sub
        auto default_qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());

        this->markers_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(markers_topic_name_, 20);

        if (pub_position_){
            this->right_step_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("step_point_right", 20);
            this->left_step_pub_  = this->create_publisher<geometry_msgs::msg::PointStamped>("step_point_left", 20);
        }
        this->right_step_topic_sub_ = this->create_subscription<walker_msgs::msg::StepStamped>(steps_topic_name_ + "_right", default_qos, std::bind(&PlotSteps::rightStepsCallback, this, std::placeholders::_1));
        this->left_step_topic_sub_  = this->create_subscription<walker_msgs::msg::StepStamped>(steps_topic_name_ + "_left",  default_qos, std::bind(&PlotSteps::leftStepsCallback,  this, std::placeholders::_1));
    }

private:

    std::string steps_topic_name_;
    std::string markers_topic_name_;
    std::string markers_namespace_;
    bool pub_position_;
    double marker_display_lifetime_;
    double speed_dead_zone_;
    double marker_size_;
    double min_confidence_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr markers_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr right_step_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr left_step_pub_;
    
    rclcpp::Subscription<walker_msgs::msg::StepStamped>::SharedPtr right_step_topic_sub_;
    rclcpp::Subscription<walker_msgs::msg::StepStamped>::SharedPtr left_step_topic_sub_;
    
    visualization_msgs::msg::Marker leg_marker;

    void rightStepsCallback(const walker_msgs::msg::StepStamped::SharedPtr stepSt){
        this->stepsCallback(1, *stepSt);
    }

    void leftStepsCallback(const walker_msgs::msg::StepStamped::SharedPtr stepSt){
        this->stepsCallback(0, *stepSt);
    }


    void stepsCallback(int id, walker_msgs::msg::StepStamped stepSt){

            if (stepSt.confidence>=min_confidence_){                
                leg_marker = fill_marker(stepSt, id);            
                markers_pub_->publish(leg_marker);
                leg_marker = fill_text_marker(stepSt, id);            
                markers_pub_->publish(leg_marker);

                if (pub_position_){
                    if (id==0)
                        this->left_step_pub_->publish(stepSt.position);
                    else
                        this->right_step_pub_->publish(stepSt.position);
                    //RCLCPP_WARN(this->get_logger(), "step plotted at: [%3.3f, %3.3f, %3.3f] [%s]", stepSt.position.point.x, stepSt.position.point.y, stepSt.position.point.z, stepSt.position.header.frame_id.c_str());    
                }
                

            } else {
                RCLCPP_WARN(this->get_logger(), "confidence [%3.3f] under min value [%3.3f]!",stepSt.confidence, min_confidence_);
            }
    }
    
    visualization_msgs::msg::Marker fill_marker(walker_msgs::msg::StepStamped stepSt, int id) {
        visualization_msgs::msg::Marker m;
        m.header = stepSt.position.header;
        m.ns = markers_namespace_;
        m.id = id;
        if (id==0)
            m.type = m.SPHERE; // left
        else
            m.type = m.CUBE; // right
        m.pose.position = stepSt.position.point;
        m.pose.position.z = 0.0;
        m.scale.x = marker_size_;
        m.scale.y = marker_size_;
        m.scale.z = marker_size_;
        m.color.a = stepSt.confidence;
        m.color.r = 0;
        m.color.g = 0;
        m.color.b = 0;  

        double v = pow(stepSt.speed.x * stepSt.speed.x + stepSt.speed.y * stepSt.speed.y + stepSt.speed.z*stepSt.speed.z, 1. / 2.);
        if (v>speed_dead_zone_){
            m.color.b = 1;
        } else if (v<(-1.0*speed_dead_zone_)){
            m.color.r = 1;
        }

        m.lifetime = rclcpp::Duration::from_seconds(marker_display_lifetime_);
        return m;
    }

    visualization_msgs::msg::Marker fill_text_marker(walker_msgs::msg::StepStamped stepSt, int id) {
        visualization_msgs::msg::Marker m;
        m.header = stepSt.position.header;
        m.ns = markers_namespace_ + "_TEXT";
        m.id = id;
        m.type = m.TEXT_VIEW_FACING;
        if (id==0)
            m.text = "left_leg";
        else
            m.text = "right_leg";
            
        m.pose.position = stepSt.position.point;
        m.pose.position.z = 0.5;
        m.scale.x = 0.05;
        m.scale.y = 0.05;
        m.scale.z = 0.05;
        m.color.a = 1.0; // Don't forget to set the alpha!
        m.color.r = 1;
        m.color.g = 1;
        m.color.b = 1;  
        m.lifetime = rclcpp::Duration::from_seconds(marker_display_lifetime_);
        return m;
    }


};

int main(int argc, char **argv)
{

    rclcpp::init(argc, argv);
    auto node = std::make_shared<PlotSteps>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
