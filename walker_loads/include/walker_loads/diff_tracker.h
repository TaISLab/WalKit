#ifndef DIFFTRACK_HH
#define DIFFTRACK_HH

#include <cmath>
#include <iostream>
#include <fstream>

#include <rclcpp/rclcpp.hpp>

// Custom Messages related Headers

#include <kalman/ExtendedKalmanFilter.hpp>
#include <kalman/DiffSystemModel.hpp>
#include <kalman/ForceMeasurementModel.hpp>
#include <kalman/SpeedMeasurementModel.hpp>
#include <kalman/FrequencyMeasurementModel.hpp>

// Some type shortcuts
typedef double T;
typedef KalmanExamples::Step::State<T> State;
typedef KalmanExamples::Step::Control<T> Control;
typedef KalmanExamples::Step::SystemModel<T> SystemModel;
typedef KalmanExamples::Step::ForceMeasurement<T> ForceMeasurement;
typedef KalmanExamples::Step::ForceMeasurementModel<T> ForceModel;
typedef KalmanExamples::Step::SpeedMeasurement<T> SpeedMeasurement;
typedef KalmanExamples::Step::SpeedMeasurementModel<T> SpeedModel;
typedef KalmanExamples::Step::FrequencyMeasurement<T> FrequencyMeasurement;
typedef KalmanExamples::Step::FrequencyMeasurementModel<T> FrequencyModel;
// Shape shared by any scalar (1-row) measurement's Jacobian/gain against our
// 8-element State -- SpeedMeasurement is just a size donor here, the actual
// type is identical for ForceMeasurement (also 1-row). Used by
// DiffTracker::partialUpdate().
typedef Kalman::Jacobian<SpeedMeasurement, State> MeasurementJacobian;
typedef Kalman::KalmanGain<State, SpeedMeasurement> MeasurementGain;

    // Tracks the left/right speed-diff and force-diff signals as two
    // sinusoids sharing one gait frequency, to smooth out the leg-load
    // ramp in PartialLoads and estimate how big the swing/stance speed gap
    // typically is for the current user (get_speed_amplitude()). See
    // kalman/DiffSystemModel.hpp for the state layout and the rationale for
    // tracking each oscillation's Cartesian in-phase/quadrature components
    // instead of amplitude+phase, kalman/FrequencyMeasurementModel.hpp for
    // how the shared frequency (w) is measured directly instead of
    // curve-fit, and partialUpdate() for why the ordinary speed/force
    // updates are barred from ever touching w. Design writeup + how to
    // reproduce this offline: see walker_loads/README.md.
    class DiffTracker{

        public:            

            DiffTracker();
            
            ~DiffTracker();

            // v0/f0: DC offsets. (va,vb)/(fa,fb): Cartesian in-phase/quadrature
            // components of the speed-diff and force-diff oscillations (see
            // kalman/DiffSystemModel.hpp for why Cartesian, not amplitude+phase).
            // w: shared angular frequency. theta: initial absolute phase.
            // w_min/w_max: physical bounds enforced on w after every predict+
            // update (see clampFrequency()) -- the speed/force measurements
            // still nudge w through weak, indirect coupling via theta between
            // the sparser, direct add_frequency_measurement() corrections;
            // without a floor, that drag alone can walk w down to ~0 and
            // stall theta's rotation entirely.
            void init(rclcpp::Node *node, std::string name, double v0, double va, double vb,
                      double f0, double fa, double fb, double w, double theta,
                      double w_min, double w_max);

            void add_speed_measurement( double speed, double ti);

            void add_force_measurement( double force, double ti);

            // w (gait angular frequency), measured externally from
            // step-alternation timing (a zero-crossing detector on the L/R
            // speed difference, see PartialLoads::steps_lc) instead of
            // relying on the force/speed models' indirect, weak coupling to
            // it -- see kalman/FrequencyMeasurementModel.hpp.
            void add_frequency_measurement( double w_estimate, double ti);

            void enable_log();

            double get_speed_diff();

            double get_force_diff();

            // amplitude of the estimated speed-diff oscillation
            // (sqrt(va^2+vb^2)): how big the swing/stance speed gap
            // typically is for the current user/session, used to scale the
            // double-support band instead of relying on one fixed absolute
            // threshold for everyone
            double get_speed_amplitude();

        private:
            void configureNoise();
            void clampFrequency();
            // Applies a speed/force measurement's correction to every state
            // except w: forces the w component of the Kalman gain to zero
            // and restores w's row/column of the covariance afterward, so
            // this measurement (whose Jacobian has zero direct sensitivity
            // to w, see kalman/DiffSystemModel.hpp) cannot move w's mean or
            // its (co)variance with anything else. Only
            // add_frequency_measurement() -- a direct observation of w --
            // still goes through the library's normal, unmasked
            // ekf_.update(), and is the only thing allowed to move it.
            void partialUpdate(const MeasurementJacobian &H, double innovation, double R);

            // Config stuff
            rclcpp::Node *node_;
            bool is_init_;
            bool is_debug_;
            std::string name_;
            double f_threshold_; // minimum valid force difference
            double w_min_;
            double w_max_;

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
            FrequencyModel freqModel_;

            // Measurements
            SpeedMeasurement speedMeas_;
            ForceMeasurement forceMeas_;
            FrequencyMeasurement freqMeas_;
            
            // State
            State ekf_state_;

    };




#endif //DIFFTRACK_HH