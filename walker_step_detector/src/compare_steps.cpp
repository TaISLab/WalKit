

#include "walker_step_detector/compare_steps.h"


        CompareSteps::CompareSteps(){}
        
        CompareSteps::CompareSteps(walker_msgs::msg::StepStamped origin){
            orig_ = origin;
        }
        
        bool operator() (const walker_msgs::msg::StepStamped &a, const walker_msgs::msg::StepStamped &b){
            float rel_dist_a = CompareSteps.dist(a,orig_);
            float rel_dist_b = CompareSteps.dist(b,orig_);
            return rel_dist_a < rel_dist_b;
        }

        static float CompareSteps::dist(const walker_msgs::msg::StepStamped &a, const walker_msgs::msg::StepStamped &o){
            float rel_dist_a = pow( pow(a.position.point.x - o.position.point.x ,2.0) +  pow(a.position.point.y - o.position.point.y ,2.0), 1. / 2.);
            return rel_dist_a;
        }