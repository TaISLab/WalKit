
#include <walker_step_detector/track_leg.h>

    TrackLeg::TrackLeg(rclcpp::Node *node_, std::string name){
        node = node_;
        t = 0;
        myfile.open (name + ".csv");
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
        
        // correct time
        rclcpp::Duration time_inc = rclcpp::Duration(u.dt());
        rclcpp::Time old_time = rclcpp::Time(curr_step.position.header.stamp);
        rclcpp::Time new_time = old_time + time_inc;
        pred_step.position.header.stamp = new_time;

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

        return pred_step;
    }


    geometry_msgs::msg::Point TrackLeg::get_speed(walker_msgs::msg::StepStamped step, walker_msgs::msg::StepStamped prev_step){
        
        double inc_x, inc_y, inc_z, inc_t, st, pst;
        geometry_msgs::msg::Point vel;
        vel.x = vel.y = vel.z = 0;

        if ((step.confidence!=0) && (prev_step.confidence!=0)){
            
            st = step.position.header.stamp.sec + step.position.header.stamp.nanosec*1e-9;
            pst = prev_step.position.header.stamp.sec + prev_step.position.header.stamp.nanosec*1e-9;

            if ( (st>0) && (pst>0) && (st!=pst) ) {
                inc_t = st - pst;

                inc_x = step.position.point.x - prev_step.position.point.x;
                inc_y = step.position.point.y - prev_step.position.point.y;
                inc_z = step.position.point.z - prev_step.position.point.z;

                vel.x = inc_x / inc_t;
                vel.y = inc_y / inc_t;
                vel.z = inc_z / inc_t;                        
            }else{
                RCLCPP_ERROR(node->get_logger(), "get_speed: Problem with stamps");    
            }
            
        } else {
            RCLCPP_ERROR(node->get_logger(), "get_speed: Problem with confidences");
        }
     
        return vel;
    }    