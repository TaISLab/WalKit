
#include <walker_loads/diff_tracker.h>
    

    DiffTracker::DiffTracker(){
        is_init_=false;        
        is_debug_=false;        
    }

    void DiffTracker::enable_log(){
        is_debug_ = true;
        debug_file_.open (name_ + "_measurements.csv");
        debug_file_  << "dt"     << ","
                     << "sp_m"    << ","
                     << "fo_m"    << ","
                     << "fw_m"    << ","
                     << "sp_pred" << ","
                     << "fo_pred" << ","
                     << "amp"     << ","
                     << "va"      << ","
                     << "vb"      << ","
                     << "w"       << ","
                     << "theta"   << std::endl;
/*
                import matplotlib.pyplot as plt  
                import pandas as pd      
                import numpy as np    

                data = pd.read_csv('diff_tracker_measurements.csv', na_values='nan')
                data['t'] = np.cumsum(data.dt)
                data['t'] = data.t - data.t[0]
                data['t'] = data['t'] 

                # % of measurements
                valid_speed = 100*np.sum(np.isfinite(data.sp_m))/len(data.sp_m)
                valid_force = 100*np.sum(np.isfinite(data.fo_m))/len(data.fo_m)

                plt.plot(data.t, data.sp_m,    color='blue',   linestyle='None',  marker='.',    label='speed diff measurement')
                plt.plot(data.t, data.sp_pred, color='orange', linestyle='solid', marker='None', label='speed diff prediction')
                plt.legend()
                plt.show()

                plt.plot(data.t, data.fo_m,    color='blue',   linestyle='None',  marker='.',    label='Force diff measurement')
                plt.plot(data.t, data.fo_pred, color='orange', linestyle='solid', marker='None', label='Force diff prediction')
                plt.legend()                
                plt.show()

*/                     
    }

    void DiffTracker::init(rclcpp::Node *node, std::string name, double v0, double va, double vb,
                           double f0, double fa, double fb, double w, double theta,
                           double w_min, double w_max){

        if (!is_init_){
            is_init_=true;
            node_ = node;
            t_ = 0;
            name_ = name;

            ekf_state_.v0()    = v0;      // meters/s
            ekf_state_.va()    = va;      // meters/s (in-phase)
            ekf_state_.vb()    = vb;      // meters/s (quadrature)
            ekf_state_.f0()    = f0;      // Kg.
            ekf_state_.fa()    = fa;      // Kg. (in-phase)
            ekf_state_.fb()    = fb;      // Kg. (quadrature)
            ekf_state_.w()     = w;       // rads/s
            ekf_state_.theta() = theta;   // rads
            f_threshold_ = 0.713;    // kg. == 7 N as stated in "On Gait Analysis Estimation Errors Using Force Sensors on a Smart Rollator"
            w_min_ = w_min;
            w_max_ = w_max;
            // Init filter with system state
            ekf_.init(ekf_state_);

            configureNoise();
        }

    }

    void DiffTracker::clampFrequency(){
        // Originally added because add_speed_measurement()/
        // add_force_measurement() dragged w towards 0 through indirect
        // coupling via theta between the sparser, direct
        // add_frequency_measurement() corrections (see kalman/DiffSystemModel.hpp).
        // Now that those two go through partialUpdate() and can no longer
        // touch w at all, that drag is gone and this should rarely if ever
        // trigger -- kept as a cheap backstop (e.g. against a bad but
        // in-range add_frequency_measurement() run of luck, or before the
        // first one ever arrives) rather than removed.
        if (ekf_state_.w() < w_min_){
            ekf_state_.w() = w_min_;
        } else if (ekf_state_.w() > w_max_){
            ekf_state_.w() = w_max_;
        }
    }

    void DiffTracker::partialUpdate(const MeasurementJacobian &H, double innovation, double R){
        // Same math as Kalman::ExtendedKalmanFilter::update(), except the
        // Kalman gain's w component is zeroed before it's applied, and w's
        // row/column of P are restored afterward. Zeroing K(W) alone only
        // guarantees row W of (K*H*P) is zero (so P's row W is untouched);
        // column W can still move through the OTHER states' gains times
        // (H*P)(W) -- which is exactly the P(*,theta)-driven leakage into
        // P(*,w) this exists to stop -- so both are restored explicitly.
        Kalman::Covariance<State> P = ekf_.getCovariance();

        double S = (H * P * H.transpose())(0,0) + R;
        MeasurementGain K = (P * H.transpose()) / S;
        K(State::W, 0) = 0.0;

        ekf_state_ += K * innovation;

        Kalman::Covariance<State> P_new = P - (K * H * P);
        for (int j = 0; j < State::RowsAtCompileTime; ++j){
            P_new(State::W, j) = P(State::W, j);
            P_new(j, State::W) = P(j, State::W);
        }

        ekf_.init(ekf_state_);
        ekf_.setCovariance(P_new);
    }

    void DiffTracker::configureNoise(){
        // None of this was ever set before: sys_/forceModel_/speedModel_/ekf_
        // all inherit Kalman::StandardBase, whose constructor defaults every
        // covariance to the identity matrix (see kalman/StandardBase.hpp).
        // A unit-variance (1.0) process/measurement noise is wildly wrong for
        // an 8-state vector mixing m/s, kg and rad units. These values are
        // physically-reasoned starting points, not empirically fitted --
        // expect to retune them once walker_step_detector gives trustworthy
        // step speeds to validate against.
        //
        // Process noise Q: how much each state is allowed to drift between
        // consecutive predict() calls (this implementation does not scale Q
        // by dt, so these are "per update call" figures, assuming updates
        // arrive a few times a second as they do from the handle/step
        // topics).
        Kalman::Covariance<State> Q = Kalman::Covariance<State>::Zero();
        Q(State::V0, State::V0)       = 0.02  * 0.02;   // v0: (0.02 m/s)^2  -- near-constant left/right bias
        Q(State::VA, State::VA)       = 0.05  * 0.05;   // va: (0.05 m/s)^2  -- swing amplitude can vary call to call
        Q(State::VB, State::VB)       = 0.05  * 0.05;   // vb: (0.05 m/s)^2
        Q(State::F0, State::F0)       = 0.05  * 0.05;   // f0: (0.05 kg)^2
        Q(State::FA, State::FA)       = 0.1   * 0.1;    // fa: (0.1 kg)^2
        Q(State::FB, State::FB)       = 0.1   * 0.1;    // fb: (0.1 kg)^2
        // w: (0.01 rad/s)^2. Tried tightening this to (0.001)^2 to resist the
        // indirect drag from add_speed_measurement()/add_force_measurement()
        // (see clampFrequency()'s comment) -- it backfired: P(W,W) collapses
        // between the rare, direct add_frequency_measurement() corrections
        // just as much as between the frequent indirect ones, so the *good*
        // corrections lost gain too (K_w ~ P(W,W)/(P(W,W)+R_w)) and w spent
        // more of the trace pinned at w_min_, not less (offline replay
        // against MF_test05: 998/1151 rows at the floor and amplitude
        // growing monotonically to 360+, versus mostly single digits with
        // this value). Q(W,W) trades off both channels at once, so it can't
        // cleanly separate "trust the rare good measurement" from "resist
        // the frequent bad one" -- left at its original value.
        Q(State::W,  State::W )       = 0.01  * 0.01;
        Q(State::THETA, State::THETA) = 1e-6;           // theta's own evolution is already deterministic in f(); this is residual model error only
        sys_.setCovariance(Q);

        // Measurement noise R: how much a single reading is trusted.
        Kalman::Covariance<SpeedMeasurement> Rs = Kalman::Covariance<SpeedMeasurement>::Zero();
        Rs(SpeedMeasurement::DV, SpeedMeasurement::DV) = 0.1 * 0.1;    // (0.1 m/s)^2
        speedModel_.setCovariance(Rs);

        Kalman::Covariance<ForceMeasurement> Rf = Kalman::Covariance<ForceMeasurement>::Zero();
        Rf(ForceMeasurement::DF, ForceMeasurement::DF) = 0.3 * 0.3;    // (0.3 kg)^2
        forceModel_.setCovariance(Rf);

        // Zero-crossing period estimates are themselves noisy (each one
        // comes from a single half-cycle timing, on top of noisy step
        // detection), but this is still w's only *direct* observation
        // channel, so it's trusted more than the process noise Q(W,W) alone
        // would imply.
        Kalman::Covariance<FrequencyMeasurement> Rw = Kalman::Covariance<FrequencyMeasurement>::Zero();
        Rw(FrequencyMeasurement::DW, FrequencyMeasurement::DW) = 0.5 * 0.5;    // (0.5 rad/s)^2
        freqModel_.setCovariance(Rw);

        // Initial state covariance P0: how much to trust the hardcoded
        // priors passed into init() before any measurement arrives. Set
        // wider than the steady-state Q above so early measurements can
        // move the state away from a poor initial guess quickly.
        Kalman::Covariance<State> P0 = Kalman::Covariance<State>::Zero();
        P0(State::V0, State::V0)       = 0.5 * 0.5;
        P0(State::VA, State::VA)       = 0.5 * 0.5;
        P0(State::VB, State::VB)       = 0.5 * 0.5;
        P0(State::F0, State::F0)       = 5.0 * 5.0;
        P0(State::FA, State::FA)       = 5.0 * 5.0;
        P0(State::FB, State::FB)       = 5.0 * 5.0;
        P0(State::W,  State::W )       = 2.0 * 2.0;   // w's initial guess is the least trustworthy of the eight
        P0(State::THETA, State::THETA) = 1.0 * 1.0;
        ekf_.setCovariance(P0);
    }
            
    DiffTracker::~DiffTracker(){
        if (is_debug_)
            debug_file_.close();
    }

    void DiffTracker::add_speed_measurement( double speed, double ti){
  
        if (is_init_){
            // Predict state for current time-step using the filters
            u_.dt() = (ti-t_)*1e-9;
            ekf_state_ = ekf_.predict(sys_, u_);
            
            // Update EKF using measurement, without letting it move w (see
            // partialUpdate()): this measurement's Jacobian has zero direct
            // sensitivity to w, so any change partialUpdate() forbade of
            // it would only ever have been indirect leakage, never signal.
            double innovation = speed - speedModel_.h(ekf_state_).dv();
            double R = speedModel_.getCovariance()(SpeedMeasurement::DV, SpeedMeasurement::DV);
            partialUpdate(speedModel_.computeJacobian(ekf_state_), innovation, R);
            clampFrequency();

            // store last prediction time
            t_ = ti;

            // save for further analysis
            if (is_debug_){
                speedMeas_ = speedModel_.h(ekf_state_);                
                forceMeas_ = forceModel_.h(ekf_state_);            
                debug_file_  << u_.dt()         << ","
                             << speed           << ","
                             << "nan"           << ","
                             << "nan"           << ","
                             << speedMeas_.dv() << ","
                             << forceMeas_.df() << ","
                             << get_speed_amplitude() << ","
                             << ekf_state_.va() << ","
                             << ekf_state_.vb() << ","
                             << ekf_state_.w()  << ","
                             << ekf_state_.theta() << std::endl;
            }
        } 

    }

    void DiffTracker::add_force_measurement( double force, double ti){
  
        if (is_init_){
            if (force>f_threshold_){
                // Predict state for current time-step using the filters
                u_.dt() = (ti-t_)*1e-9;
                ekf_state_ = ekf_.predict(sys_, u_);
                
                // Update EKF using measurement, without letting it move w
                // (see partialUpdate()) -- same reasoning as add_speed_measurement().
                double innovation = force - forceModel_.h(ekf_state_).df();
                double R = forceModel_.getCovariance()(ForceMeasurement::DF, ForceMeasurement::DF);
                partialUpdate(forceModel_.computeJacobian(ekf_state_), innovation, R);
                clampFrequency();

                // store last prediction time
                t_ = ti;

                // save for further analysis
                if (is_debug_){
                    speedMeas_ = speedModel_.h(ekf_state_);                
                    forceMeas_ = forceModel_.h(ekf_state_);
                    
                    debug_file_  << u_.dt()         << ","
                                << "nan"           << ","
                                << force           << ","
                                << "nan"           << ","
                                << speedMeas_.dv() << ","
                                << forceMeas_.df() << ","
                                << get_speed_amplitude() << ","
                                << ekf_state_.va() << ","
                                << ekf_state_.vb() << ","
                                << ekf_state_.w()  << ","
                                << ekf_state_.theta() << std::endl;
                }
            } else{
              RCLCPP_DEBUG(node_->get_logger(), "Force measurement (%3.3f) is under threshold (%3.3f)", force, f_threshold_);
            }
        }

    }

    void DiffTracker::add_frequency_measurement( double w_estimate, double ti){

        if (is_init_){
            // Predict state for current time-step using the filters
            u_.dt() = (ti-t_)*1e-9;
            ekf_state_ = ekf_.predict(sys_, u_);

            // Update EKF using measurement
            freqMeas_.dw() = w_estimate;
            ekf_state_ = ekf_.update(freqModel_, freqMeas_);
            clampFrequency();

            // store last prediction time
            t_ = ti;

            // save for further analysis
            if (is_debug_){
                speedMeas_ = speedModel_.h(ekf_state_);
                forceMeas_ = forceModel_.h(ekf_state_);
                debug_file_  << u_.dt()         << ","
                             << "nan"           << ","
                             << "nan"           << ","
                             << w_estimate      << ","
                             << speedMeas_.dv() << ","
                             << forceMeas_.df() << ","
                             << get_speed_amplitude() << ","
                             << ekf_state_.va() << ","
                             << ekf_state_.vb() << ","
                             << ekf_state_.w()  << ","
                             << ekf_state_.theta() << std::endl;
            }
        }

    }

    double DiffTracker::get_force_diff(){
        
        if (is_init_){
            forceMeas_ = forceModel_.h(ekf_state_);

            RCLCPP_DEBUG(node_->get_logger(), "Predicted force diff: (%3.3f)", forceMeas_.df());            

            return forceMeas_.df();
        }
        return NULL;
    }

    double DiffTracker::get_speed_diff(){

        if (is_init_){
            speedMeas_ = speedModel_.h(ekf_state_);

            if (is_debug_){
                RCLCPP_DEBUG(node_->get_logger(), "Predicted force diff: (%3.3f)", speedMeas_.dv());
            }

            return speedMeas_.dv();
        }
        return NULL;
    }

    double DiffTracker::get_speed_amplitude(){

        if (is_init_){
            return std::sqrt(ekf_state_.va()*ekf_state_.va() + ekf_state_.vb()*ekf_state_.vb());
        }
        return NULL;
    }