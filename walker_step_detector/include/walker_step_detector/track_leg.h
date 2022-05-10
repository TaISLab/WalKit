#ifndef TRACKLEG_HH
#define TRACKLEG_HH

#include<set>
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

            TrackLeg(rclcpp::Node *node_);
            
            ~TrackLeg();

            void add( walker_msgs::msg::StepStamped step);

            walker_msgs::msg::StepStamped get_step();

            int size();

            walker_msgs::msg::StepStamped predict_step(double t);

            geometry_msgs::msg::Point get_speed(walker_msgs::msg::StepStamped step, walker_msgs::msg::StepStamped prev_step);

        private:
            // vector where we store laser detections that could be an step detection
            std::vector<walker_msgs::msg::StepStamped> step_list;

            walker_msgs::msg::StepStamped curr_step;

            // last update time
            double t;

            rclcpp::Node *node;

            // Extended Kalman Filter
            Kalman::ExtendedKalmanFilter<State> ekf;

            // System model
            SystemModel sys;
    
            // Measurement model
            PositionModel pm;

    };




#endif
