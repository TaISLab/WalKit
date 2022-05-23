#ifndef TRACKLEG_HH
#define TRACKLEG_HH

#include<set>
#include <iostream>
#include <fstream>

#include <rclcpp/rclcpp.hpp>

// Custom Messages related Headers
#include "walker_msgs/msg/step_stamped.hpp"

#include "kalman/ExtendedKalmanFilter.hpp"
#include "kalman/UnscentedKalmanFilter.hpp"

#include "kalman/SystemModelLeg.hpp"
#include "kalman/PositionMeasurementModelLeg.hpp"
#include "walker_step_detector/compare_steps.h"


// Some type shortcuts
typedef float T;
typedef Leg::State<T> State;
typedef Leg::Control<T> Control;
typedef Leg::SystemModel<T> SystemModel;

typedef Leg::PositionMeasurement<T> PositionMeasurement;
typedef Leg::PositionMeasurementModel<T> PositionModel;

    class TrackLeg{   

        public:            

            TrackLeg();
            
            ~TrackLeg();

            void init(rclcpp::Node *node_, std::string name, double d0, double a0, double f0, double p0);

            void add( walker_msgs::msg::StepStamped step);

            walker_msgs::msg::StepStamped get_step();

            int size();

            walker_msgs::msg::StepStamped predict_step(double t);

            geometry_msgs::msg::Point get_speed(walker_msgs::msg::StepStamped step, walker_msgs::msg::StepStamped prev_step);
            
            void enable_log();
        private:
            std::ofstream myfile;

            // vector where we store laser detections that could be an step detection
            std::vector<walker_msgs::msg::StepStamped> step_list;

            walker_msgs::msg::StepStamped curr_step;

            // last update time
            double t;

            rclcpp::Node *node;

            bool is_init;
            bool is_debug;

            std::string name;

            // Extended Kalman Filter
            Kalman::ExtendedKalmanFilter<State> ekf;

            // System model
            SystemModel sys;
    
            // Measurement model
            PositionModel pm;

    };




#endif
