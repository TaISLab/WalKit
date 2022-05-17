#include <walker_step_detector/legs_tracker.h>

    LegsTracker::LegsTracker(rclcpp::Node *node_):l_tracker(node_, "left"),r_tracker(node_, "right"){
        node = node_;
    }

    LegsTracker::~LegsTracker(){        
    }

    void LegsTracker::add_detections( std::list<walker_msgs::msg::StepStamped> detect_steps){

        walker_msgs::msg::StepStamped first_step, last_step;

        unsigned int n_points;

        n_points = detect_steps.size();
        
        if (n_points==1){
            first_step = detect_steps.front();
            // left should have y<0
            if (first_step.position.point.y<0) {
                l_tracker.add(first_step);
            } else{
                r_tracker.add(first_step);
            }            
        } else if (n_points==2) {
            first_step = detect_steps.front();
            last_step = detect_steps.back();

            // left should have lower y
            if (first_step.position.point.y<last_step.position.point.y) {
                l_tracker.add(first_step);
                r_tracker.add(last_step);
            } else{
                l_tracker.add(last_step);
                r_tracker.add(first_step);                
            }            
        } else{ // 3 points at least ...
            detect_steps.sort(CompareSteps()); // sorted left-right
            // pick most extreme ones
            first_step = detect_steps.front();
            l_tracker.add(first_step);
            last_step = detect_steps.back();

            // add all to kalman and let it smooth it
            // while(detect_steps.size()>0){  
            //     first_step = detect_steps.front();
            //     detect_steps.pop_front();
            //     l_tracker.add(first_step);
            //     if (detect_steps.size()>0){
            //         last_step = detect_steps.back();
            //         detect_steps.pop_back();
            //         r_tracker.add(last_step);
            //     }
            // }
        }

    }

    void LegsTracker::get_steps(walker_msgs::msg::StepStamped* step_r, walker_msgs::msg::StepStamped* step_l, double t){
        
        //RCLCPP_ERROR (node->get_logger(), "Prediction requested at time (%3.3f)",t*1e-9);
        *step_r = r_tracker.predict_step(t);
        *step_l = l_tracker.predict_step(t);

        RCLCPP_ERROR (node->get_logger(), "Pred Left step at  [%3.3f, %3.3f]", step_l->position.point.x, step_l->position.point.y);
        RCLCPP_ERROR (node->get_logger(), "Pred Right step at [%3.3f, %3.3f]", step_r->position.point.x, step_r->position.point.y);
        RCLCPP_ERROR (node->get_logger(), "xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx\n\n\n\n");

    }




