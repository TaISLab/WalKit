

#include "walker_step_detector/compare_steps.h"


        CompareSteps::CompareSteps(){}
        
        // CompareSteps::CompareSteps(walker_msgs::msg::StepStamped origin){
        //     orig_ = origin;
        // }
        
        // bool  CompareSteps::operator() (walker_msgs::msg::StepStamped a, walker_msgs::msg::StepStamped b){
        //     float rel_dist_a = this->dist(a,orig_);
        //     float rel_dist_b = this->dist(b,orig_);
        //     return rel_dist_a < rel_dist_b;
        // }

        float CompareSteps::dist( walker_msgs::msg::StepStamped a, walker_msgs::msg::StepStamped o){
            float rel_dist_a = pow( pow(a.position.point.x - o.position.point.x ,2.0) +  pow(a.position.point.y - o.position.point.y ,2.0), 1. / 2.);
            return rel_dist_a;
        }