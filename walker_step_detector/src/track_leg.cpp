
#include <walker_step_detector/track_leg.h>

    TrackLeg::TrackLeg();
            
    ~TrackLeg::TrackLeg();

    void TrackLeg::add( walker_msgs::msg::StepStamped step){
        step_set.insert(step);
    }

    walker_msgs::msg::StepStamped TrackLeg::get_step(){
        return curr_step;
    }

    int TrackLeg::size(){
        return step_set.size();
    }

    walker_msgs::msg::StepStamped TrackLeg::predict_step(double ti){

        walker_msgs::msg::StepStamped pred_step, measure_step;
        

        // Predict state for current time-step using the filters
        u.dt() = ti-t;

        auto ekf_state = ekf.predict(sys, u);

        // option 1: add closest detection to kalman filter
        if (step_set.size() > 0) {
            measure_step = *std::next(step_set.begin(), 0);

            PositionMeasurement position;
            position.pos_x() = measure_step.position.point.x;
            position.pos_y() = measure_step.position.point.x;        

            // Update EKF using measurement
            ekf_state = ekf_x.update(pm, position);
        }

        PositionMeasurement pred_position = pm.h(ekf_state);

        pred_step.position.point.x = pred_position.pos_x();
        pred_step.position.point.y = pred_position.pos_y();
        pred_step.position.header = curr_step.position.header;
        
        // correct time



        // get speeds
        pred_step.speed = get_speed(pred_step, curr_step);
        
        // are they tracked or just predicted ?
        pred_step.tracked = ;
        pred_step.confidence = ;


        // cleaning: my new reference is this one
        curr_step = pred_step;

        // new set will be built around new center
        step_set = std::set<walker_msgs::msg::StepStamped, CompareSteps(curr_step)>;

        return pred_step;
    }




    geometry_msgs::msg::Point get_speed(walker_msgs::msg::StepStamped step, walker_msgs::msg::StepStamped prev_step){
        
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
                printf("")    
            }
            
        } else {
            printf("")
        }
     
        return vel;
    }    