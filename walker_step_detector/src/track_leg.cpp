
#include <walker_step_detector/track_leg.h>

    TrackLeg::TrackLeg(rclcpp::Node *node_, std::string name){
        node = node_;
        t = 0;

        // System initial state
        State x0;

        x0.d_x() = 0.01;   // meters
        x0.a_x() = 0.15;   // meters
        x0.f_x() = 0.75;  // hertzs
        x0.p_x() = 0;     // rads

        x0.d_y() = 0.01;   // meters
        x0.a_y() = 0.15;   // meters
        x0.f_y() = 0.75;  // hertzs
        x0.p_y() = 0;     // rads

        // Init filter with system state
        ekf.init(x0);    

        myfile.open (name + ".csv");

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
        myfile.close();
    }

    void TrackLeg::add( walker_msgs::msg::StepStamped step){
        step_list.push_back(step);
    }

    walker_msgs::msg::StepStamped TrackLeg::get_step(){
        return curr_step;
    }

    int TrackLeg::size(){
        return step_list.size();
    }

    walker_msgs::msg::StepStamped TrackLeg::predict_step(double ti){

        walker_msgs::msg::StepStamped pred_step, measure_step;
        // Control input
        Control u;

        // is it a tracked measurement or just a prediction?
        pred_step.tracked = false;
        // how sure are we this is a "leg"
        pred_step.confidence = curr_step.confidence;
        
        // Predict state for current time-step using the filters
        u.dt() = (ti-t)*1e-9;

        auto ekf_state = ekf.predict(sys, u);

        // option 1: consider closest detection 
        //           to current step position as
        //           kalman filter measurement
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

            PositionMeasurement position;
            position.pos_x() = measure_step.position.point.x;
            position.pos_y() = measure_step.position.point.y;        

            // Update EKF using measurement
            ekf_state = ekf.update(pm, position);
            pred_step.tracked = true;   
            // u,x,y   // predict + update      
            myfile   << u.dt()     << "," << position.pos_x() << "," << position.pos_y() << std::endl;
        } else{
            // u,nan,nan // predict -
            myfile   << u.dt()     << "," << "nan" << "," << "nan" << std::endl;
            RCLCPP_ERROR(node->get_logger(), "No laser measurement available. No EKF update.");
        }

        PositionMeasurement pred_position = pm.h(ekf_state);

        pred_step.position.point.x = pred_position.pos_x();
        pred_step.position.point.y = pred_position.pos_y();
        pred_step.position.header = curr_step.position.header;
        
        // set prediction time
        pred_step.position.header.stamp = rclcpp::Time(ti);

        // get speeds
        pred_step.speed = get_speed(pred_step, curr_step);
        
        // frame reference
        pred_step.position.header.frame_id = curr_step.position.header.frame_id;

        // cleaning: my new reference is this one
        curr_step = pred_step;

        // new list, forget previous potential detections ...
        step_list.clear();     

        // store last prediction time    
        t = ti;

        RCLCPP_ERROR (node->get_logger(), "Pred -- step at  [%3.3f, %3.3f]", pred_step.position.point.x, pred_step.position.point.y);
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
                RCLCPP_ERROR(node->get_logger(), "get_speed: step timestamp == 0 ");    
                return vel;
        }
        if (pst==0){
                RCLCPP_ERROR(node->get_logger(), "get_speed: prev step timestamp == 0 ");    
                return vel;
        }
        if ( st == pst){
                RCLCPP_ERROR(node->get_logger(), "get_speed: both timestamps are equal ");    
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