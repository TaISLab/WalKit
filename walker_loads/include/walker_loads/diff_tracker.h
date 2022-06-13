#ifndef DIFFTRACK_HH
#define DIFFTRACK_HH

#include <iostream>
#include <fstream>

#include <rclcpp/rclcpp.hpp>

// Custom Messages related Headers

#include <kalman/ExtendedKalmanFilter.hpp>
#include <kalman/DiffSystemModel.hpp>
#include <kalman/ForceMeasurementModel.hpp>
#include <kalman/SpeedMeasurementModel.hpp>

// Some type shortcuts
typedef double T;
typedef KalmanExamples::Step::State<T> State;
typedef KalmanExamples::Step::Control<T> Control;
typedef KalmanExamples::Step::SystemModel<T> SystemModel;
typedef KalmanExamples::Step::ForceMeasurement<T> ForceMeasurement;
typedef KalmanExamples::Step::ForceMeasurementModel<T> ForceModel;
typedef KalmanExamples::Step::SpeedMeasurement<T> SpeedMeasurement;
typedef KalmanExamples::Step::SpeedMeasurementModel<T> SpeedModel;

    class DiffTracker{   

        public:            

            DiffTracker();
            
            ~DiffTracker();

            void init(rclcpp::Node *node, std::string name, double v0, double v1, double f0, double f1, double w, double d, double vp);

            void add_speed_measurement( double speed, double ti);

            void add_force_measurement( double force, double ti);

            void enable_log();

            double get_speed_diff();

        private:
            // Config stuff
            rclcpp::Node *node_;
            bool is_init_;
            bool is_debug_;
            std::string name_;

            // debug file to check kalman working
            std::ofstream debug_file_;

            // last update time
            double t_;

            // Extended Kalman Filter
            Kalman::ExtendedKalmanFilter<State> ekf_;

            // System model
            SystemModel sys_;
    
            // Control input
            Control u_;    

            // Measurement models
            ForceModel forceModel_;
            SpeedModel speedModel_;

            // Measurements
            SpeedMeasurement speedMeas_;
            ForceMeasurement forceMeas_;
            
            // State
            State ekf_state_;

    };




#endif //DIFFTRACK_HH