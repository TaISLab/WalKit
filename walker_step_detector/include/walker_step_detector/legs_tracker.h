#ifndef LEGSTRACKER_HH
#define LEGSTRACKER_HH


#include <rclcpp/rclcpp.hpp>
// Custom Messages related Headers
#include "walker_msgs/msg/step_stamped.hpp"
#include <walker_step_detector/track_leg.h>


    class LegsTracker{   

        public:
            LegsTracker();
                        
            ~LegsTracker();

            void init(rclcpp::Node *node_,double d0, double a0, double f0, double p0);
            
            void add_detections( std::list<walker_msgs::msg::StepStamped> detect_steps);

            void get_steps(walker_msgs::msg::StepStamped* step_r, walker_msgs::msg::StepStamped* step_l, double t);
            void enable_log();
            bool is_init;
        private:
            TrackLeg l_tracker;
            TrackLeg r_tracker;
            rclcpp::Node *node;
            
            bool is_debug;
    };




#endif
