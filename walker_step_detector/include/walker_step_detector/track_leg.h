#ifndef TRACKLEG_HH
#define TRACKLEG_HH


// Custom Messages related Headers
#include "walker_msgs/msg/step_stamped.hpp"

#include "walker_step_detector/SystemModelLeg.hpp"
#include "walker_step_detector/PositionMeasurementModelLeg.hpp"

#include <walker_step_detector/ExtendedKalmanFilter.hpp>
#include <walker_step_detector/UnscentedKalmanFilter.hpp>


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

            void add( walker_msgs::msg::StepStamped step);

            walker_msgs::msg::StepStamped get_step();

            int size();

            walker_msgs::msg::StepStamped TrackLeg::predict_step(double t);

        private:
            // set where we store laser detections that could be an step detection
            std::set<walker_msgs::msg::StepStamped, CompareSteps> step_set;

            walker_msgs::msg::StepStamped curr_step;

            // last update time
            double t;


            // Extended Kalman Filter
            Kalman::ExtendedKalmanFilter<State> ekf;

            // System model
            SystemModel sys;
    
            // Measurement model
            PositionModel pm;

    };




#endif
