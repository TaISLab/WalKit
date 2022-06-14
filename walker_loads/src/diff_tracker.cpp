
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
                     << "sp_pred" << "," 
                     << "fo_pred" << std::endl;      
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

    void DiffTracker::init(rclcpp::Node *node, std::string name, double v0, double v1, double f0, double f1, double w, double d, double vp){

        if (!is_init_){
            is_init_=true;
            node_ = node;
            t_ = 0;
            name_ = name;

            ekf_state_.v0()  = v0;   // meters/s
            ekf_state_.v1()  = v1;   // meters/s
            ekf_state_.f0()  = f0;   // Kg.
            ekf_state_.f1()  = f1;   // Kg.
            ekf_state_.w()   = w;    // rads/s
            ekf_state_.d()   = d;    // rads
            ekf_state_.vp()  = vp;   // rads
            f_threshold_ = 0.713;    // kg. == 7 N as stated in "On Gait Analysis Estimation Errors Using Force Sensors on a Smart Rollator"
            // Init filter with system state
            ekf_.init(ekf_state_);    
        }

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
            
            // Update EKF using measurement
            speedMeas_.dv() = speed;
            ekf_state_ = ekf_.update(speedModel_, speedMeas_);       

            // store last prediction time    
            t_ = ti;

            // save for further analysis
            if (is_debug_){
                speedMeas_ = speedModel_.h(ekf_state_);                
                forceMeas_ = forceModel_.h(ekf_state_);            
                debug_file_  << u_.dt()         << "," 
                             << speed           << "," 
                             << "nan"           << "," 
                             << speedMeas_.dv() << "," 
                             << forceMeas_.df() << std::endl;                
            }
        } 

    }

    void DiffTracker::add_force_measurement( double force, double ti){
  
        if (is_init_){
            if (force>f_threshold_){
                // Predict state for current time-step using the filters
                u_.dt() = (ti-t_)*1e-9;
                ekf_state_ = ekf_.predict(sys_, u_);
                
                // Update EKF using measurement
                forceMeas_.df() = force;
                ekf_state_ = ekf_.update(forceModel_, forceMeas_);       

                // store last prediction time    
                t_ = ti;

                // save for further analysis
                if (is_debug_){
                    speedMeas_ = speedModel_.h(ekf_state_);                
                    forceMeas_ = forceModel_.h(ekf_state_);
                    
                    debug_file_  << u_.dt()         << "," 
                                << "nan"           << "," 
                                << force           << "," 
                                << speedMeas_.dv() << "," 
                                << forceMeas_.df() << std::endl;
                }
            } else{
              RCLCPP_DEBUG(node_->get_logger(), "Force measurement (%3.3f) is under threshold (%3.3f)", force, f_threshold_);            
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