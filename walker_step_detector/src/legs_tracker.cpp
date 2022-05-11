#include <walker_step_detector/legs_tracker.h>


    // LegsTracker::LegsTracker(){

    // }

    LegsTracker::LegsTracker(rclcpp::Node *node_):l_tracker(node_, "left"),r_tracker(node_, "right"){
        node = node_;
    }

    LegsTracker::~LegsTracker(){        
    }

    void LegsTracker::add_detection( geometry_msgs::msg::PointStamped position, float probability){

        walker_msgs::msg::StepStamped new_step, right_step, left_step;
        double d,dr,dl;
        new_step.position = position;
        new_step.confidence = probability;

        right_step = r_tracker.get_step();
        left_step = l_tracker.get_step();
        RCLCPP_ERROR (node->get_logger(), "Detection at  [%3.3f, %3.3f]", position.point.x, position.point.y);
        RCLCPP_ERROR (node->get_logger(), "Left step at  [%3.3f, %3.3f]", left_step.position.point.x, left_step.position.point.y);
        RCLCPP_ERROR (node->get_logger(), "Right step at [%3.3f, %3.3f]", right_step.position.point.x, right_step.position.point.y);

        d = CompareSteps::dist(right_step, left_step);
        RCLCPP_ERROR (node->get_logger(), "Left-Right leg dist     [%3.3f]", d);
        if (d>0) {
            dl = CompareSteps::dist(new_step, left_step);
            dr = CompareSteps::dist(new_step, right_step);
            RCLCPP_ERROR (node->get_logger(), "Point to left leg dist  [%3.3f]", dl);
            RCLCPP_ERROR (node->get_logger(), "Point to right leg dist [%3.3f]", dr);
            // add to closer feet
            if (dl < dr){
                RCLCPP_ERROR (node->get_logger(), "Add to left tracker");
                l_tracker.add(new_step);
            } else{
                RCLCPP_ERROR (node->get_logger(), "Add to right tracker");
                r_tracker.add(new_step);
            }
        } else { // both feet "are" at the same place
            // left should have y<0
            if (new_step.position.point.y<0) {
                RCLCPP_ERROR (node->get_logger(), "Add to left tracker");
                l_tracker.add(new_step);
            } else if (new_step.position.point.y>0) {
                RCLCPP_ERROR (node->get_logger(), "Add to right tracker");
                r_tracker.add(new_step);
            } else{
                // gosh, y is 0, and feets are in 0. Add it to smaller set!
                if (l_tracker.size() > r_tracker.size()) {
                    RCLCPP_ERROR (node->get_logger(), "Add to right tracker");
                    r_tracker.add(new_step);
                } else{
                    RCLCPP_ERROR (node->get_logger(), "Add to left tracker");
                    l_tracker.add(new_step);
                }
            }
        }
        RCLCPP_ERROR (node->get_logger(), "..................................................\n\n");
    }

    void LegsTracker::get_steps(walker_msgs::msg::StepStamped* step_r, walker_msgs::msg::StepStamped* step_l, double t){
        
        RCLCPP_ERROR (node->get_logger(), "Prediction requested at time (%3.3f)",t*1e-9);
        *step_r = r_tracker.predict_step(t);
        *step_l = l_tracker.predict_step(t);

        RCLCPP_ERROR (node->get_logger(), "Pred Left step at  [%3.3f, %3.3f]", step_l->position.point.x, step_l->position.point.y);
        RCLCPP_ERROR (node->get_logger(), "Pred Right step at [%3.3f, %3.3f]", step_r->position.point.x, step_r->position.point.y);
        RCLCPP_ERROR (node->get_logger(), "xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx\n\n\n\n");

    }




