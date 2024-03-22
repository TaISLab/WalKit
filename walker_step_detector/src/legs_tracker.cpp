#include <walker_step_detector/legs_tracker.h>

    LegsTracker::LegsTracker(){
        is_init = false;
        is_debug = false;
        status_ = false;
    }

    void LegsTracker::init(rclcpp::Node *node_,double d0, double a0, double f0, double p0){
        if (!is_init){
            is_init = true;
            node = node_;
            l_tracker.init(node_, "left", d0, a0, f0, p0);
            r_tracker.init(node_, "right",d0, a0, f0, p0);
        }
    }

    void LegsTracker::enable_log(){
        is_debug = true;
        l_tracker.enable_log();
        r_tracker.enable_log();
    }


    LegsTracker::~LegsTracker(){        
    }

    void LegsTracker::add_detections( std::list<walker_msgs::msg::StepStamped> detect_steps){

        walker_msgs::msg::StepStamped first_step, last_step;

        unsigned int n_points;

        if (is_debug){
            RCLCPP_DEBUG (node->get_logger(), "Adding (%ld) detections", detect_steps.size());
        }

        if (is_init){
            n_points = detect_steps.size();
            
            if (n_points==1){
                first_step = detect_steps.front();
                // left should have y>0
                if (first_step.position.point.y>0) {
                    l_tracker.add(first_step);
                } else{
                    r_tracker.add(first_step);
                }            
            } else if (n_points==2) {
                first_step = detect_steps.front();
                last_step = detect_steps.back();

                // left should have bigger y
                if (first_step.position.point.y>last_step.position.point.y) {
                    l_tracker.add(first_step);
                    r_tracker.add(last_step);
                } else{
                    l_tracker.add(last_step);
                    r_tracker.add(first_step);                
                }            
            } else if (n_points>2) { // 3 points at least ...
                detect_steps.sort(CompareSteps()); // sorted right-left

                // TODO: add an option to decide how to include multiple detections for each leg.

                // Option 1: pick most extreme ones
                //first_step = detect_steps.front();
                //r_tracker.add(first_step);
                //last_step = detect_steps.back();
                //l_tracker.add(last_step);
                
                // Option 2: add all to kalman and let it smooth it
                while(detect_steps.size()>0){  
                    first_step = detect_steps.front();
                    detect_steps.pop_front();
                    r_tracker.add(first_step);
                    if (detect_steps.size()>0){
                        last_step = detect_steps.back();
                        detect_steps.pop_back();
                        l_tracker.add(last_step);
                    }
                }


            }
        }
    }

    void LegsTracker::set_status(bool new_status){
        status_ = new_status;
    }

    void LegsTracker::get_steps(walker_msgs::msg::StepStamped* step_r, walker_msgs::msg::StepStamped* step_l, double t){
        
        if (status_){
            if (is_debug){
                RCLCPP_DEBUG (node->get_logger(), "Prediction requested at time (%3.3f)",t*1e-9);
            }

            *step_r = r_tracker.predict_step(t);
            *step_l = l_tracker.predict_step(t);

        } else {
            if (is_debug){
                RCLCPP_DEBUG (node->get_logger(), "Last data requested !");
            }

            *step_r = r_tracker.last_data();
            *step_l = l_tracker.last_data();        


        }


    }


    unsigned int LegsTracker::data_size(){
        unsigned int stored_steps;
        stored_steps = 0;
        if (is_init){
             stored_steps = r_tracker.size() + l_tracker.size();
             stored_steps = stored_steps >> 2;
        }

        return stored_steps;
    }


