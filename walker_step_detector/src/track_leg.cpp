
#include <walker_step_detector/track_leg.h>
    

    TrackLeg::TrackLeg(){
        is_init=false;        
        is_debug=false;        
    }

    void TrackLeg::enable_log(){
        is_debug = true;
        myfile.open (name + "_measurements.csv");
    }

    void TrackLeg::init(rclcpp::Node *node_, std::string name_, double d0, double a0, double f0, double p0){

        if (!is_init){
            is_init=true;
            node = node_;
            t = 0;
            name = name_;
            // System initial state
            State x0;

            x0.d_x() = d0;   // meters
            x0.a_x() = a0;   // meters
            x0.f_x() = f0;  // hertzs
            x0.p_x() = p0;  // rads

            x0.d_y() = d0;   // meters
            x0.a_y() = a0;   // meters
            x0.f_y() = f0;  // hertzs
            x0.p_y() = p0;  // rads

            // Init filter with system state
            ekf.init(x0);    
        }

        /*
                import matplotlib.pyplot as plt  
                import pandas as pd      
                import numpy as np    

                data = pd.read_csv('left.csv', na_values='nan', names=['dt', 'x','y'])       
                data['t'] = np.cumsum(data.dt)

                data = pd.read_csv('right.csv', na_values='nan', names=['dt', 'x','y'])
                data['t'] = np.cumsum(data.dt)

                plt.plot(data.t, data.x); plt.show()
                100*np.sum(np.isfinite(data.x))/len(data.x)


         */
    }

    // TrackLeg::TrackLeg(){
    //     t = 0;
    // }
            
    TrackLeg::~TrackLeg(){
        if (is_debug)
            myfile.close();
    }

    void TrackLeg::add( walker_msgs::msg::StepStamped step){
        if (is_init){
            step_list.push_back(step);
            //RCLCPP_ERROR (node->get_logger(), "(%s) has new step  [%3.3f, %3.3f, (%s)]",name.c_str(), step.position.point.x, step.position.point.y, step.position.header.frame_id.c_str());
        } 

    }

    walker_msgs::msg::StepStamped TrackLeg::get_step(){
        return curr_step;
    }

    int TrackLeg::size(){
        return step_list.size();
    }

    walker_msgs::msg::StepStamped TrackLeg::predict_step(double ti){

        walker_msgs::msg::StepStamped pred_step, measure_step;
        if (is_init){
            // Control input
            Control u;

            // is it a tracked measurement or just a prediction?
            pred_step.tracked = false;
            // how sure are we this is a "leg"
            pred_step.confidence = curr_step.confidence;
            pred_step.position.point.z = curr_step.position.point.z;
            
            // where?
            pred_step.position.header = curr_step.position.header;

            // Predict state for current time-step using the filters
            u.dt() = (ti-t)*1e-9;

            auto ekf_state = ekf.predict(sys, u);

            // option 1: consider closest detection 
            //           to current step position as
            //           kalman filter measurement

            if (is_debug)
                RCLCPP_DEBUG(node->get_logger(), "%d measurements available for EFK update", step_list.size());

            if (step_list.size() > 0) {
                // find closest to current step position
                double d = 99999;
                double min_dist = 99999;
                for (auto st : step_list) {
                    d = CompareSteps::dist(curr_step, st);
                    if (d<min_dist){
                        min_dist = d;
                        measure_step = st;
                    }
                }
                
                pred_step.confidence = measure_step.confidence;
                pred_step.position.header = measure_step.position.header;
                                
                PositionMeasurement position;
                position.pos_x() = measure_step.position.point.x;
                position.pos_y() = measure_step.position.point.y;        
                pred_step.position.point.z = measure_step.position.point.z;

                // Update EKF using measurement
                ekf_state = ekf.update(pm, position);
                pred_step.tracked = true;   
                // u,x,y   // predict + update      
                if (is_debug)
                    myfile   << u.dt()     << "," << position.pos_x() << "," << position.pos_y() << std::endl;
            } else{
                // u,nan,nan // predict -
                if (is_debug){
                    myfile   << u.dt()     << "," << "nan" << "," << "nan" << std::endl;
                    RCLCPP_DEBUG(node->get_logger(), "No laser measurement available. No EKF update.");
                }
            }



            PositionMeasurement pred_position = pm.h(ekf_state);

            pred_step.position.point.x = pred_position.pos_x();
            pred_step.position.point.y = pred_position.pos_y();
            
            
            // set prediction time
            pred_step.position.header.stamp = rclcpp::Time(ti);

            // get speeds
            pred_step.speed = get_speed(pred_step, curr_step);
            
            // cleaning: my new reference is this one
            curr_step = pred_step;

            // new list, forget previous potential detections ...
            step_list.clear();     

            // store last prediction time    
            t = ti;
        }
        if (is_debug){
            RCLCPP_DEBUG(node->get_logger(), "Pred (%s) step at  [%3.3f, %3.3f, (%s)]",name.c_str(), pred_step.position.point.x, pred_step.position.point.y, pred_step.position.header.frame_id.c_str());
        }
        return pred_step;
    }


    geometry_msgs::msg::Point TrackLeg::get_speed(walker_msgs::msg::StepStamped step, walker_msgs::msg::StepStamped prev_step){
        
        double inc_x, inc_y, inc_z, inc_t, st, pst;
        geometry_msgs::msg::Point vel;
        vel.x = vel.y = vel.z = 0;

        // check confidence problems
        // if (step.confidence==0) {
        //         RCLCPP_ERROR(node->get_logger(), "get_speed: step confidence == 0 ");    
        //         return vel;
        // }
        // if (prev_step.confidence==0){
        //         RCLCPP_ERROR(node->get_logger(), "get_speed: prev step confidence == 0 ");    
        //         return vel;
        // }

        // check timestamp problems
        st = step.position.header.stamp.sec + step.position.header.stamp.nanosec*1e-9;
        pst = prev_step.position.header.stamp.sec + prev_step.position.header.stamp.nanosec*1e-9;

        if (st==0) {
                RCLCPP_WARN(node->get_logger(), "get_speed: step timestamp == 0 ");    
                return vel;
        }
        if (pst==0){
                RCLCPP_WARN(node->get_logger(), "get_speed: prev step timestamp == 0 ");    
                return vel;
        }
        if ( st == pst){
                RCLCPP_WARN(node->get_logger(), "get_speed: both timestamps are equal ");    
                return vel;
        }

        // data is sane.
        inc_t = st - pst;

        inc_x = step.position.point.x - prev_step.position.point.x;
        inc_y = step.position.point.y - prev_step.position.point.y;
        inc_z = step.position.point.z - prev_step.position.point.z;

        vel.x = inc_x / inc_t;
        vel.y = inc_y / inc_t;
        vel.z = inc_z / inc_t;                        
     
        return vel;
    }    