
#include <walker_step_detector/legs_tracker.h>

    LegsTracker::LegsTracker(){
            /* constructor:
            - Crear 4 filtros: dx,dy, ix,iy
            - Posiciones iniciales en 0


            */

    }

    LegsTracker::~LegsTracker(){
        
    }

    void LegsTracker::add_detection( geometry_msgs::msg::PointStamped position, float probability){

        walker_msgs::msg::StepStamped new_step, right_step, left_step;
        new_step.position = position;
        new_step.confidence = probability_of_leg;

        right_step = r_tracker.get_step();
        left_step = l_tracker.get_step();

        if (CompareSteps.dist(right_step, left_step)>0) {
            // add to closer feet
            if (CompareSteps.dist(new_step, left_step) < CompareSteps.dist(new_step, right_step)){
                l_tracker.add(new_step);
            } else{
                r_tracker.add(new_step);
            }
        } else { // both feet "are" at the same place
            // left should have y<0
            if (new_step.position.point.y<0) {
                l_tracker.add(new_step);
            } else if (new_step.position.point.y>0) {
                r_tracker.add(new_step);
            } else{
                // gosh, y is 0, and feets are in 0. Add it to smaller set!
                if (l_tracker.size() > r_tracker.size()) {
                    r_tracker.add(new_step);
                } else{
                    l_tracker.add(new_step);
                }
            }
        }

    }

    void LegsTracker::get_steps(walker_msgs::msg::StepStamped* step_r, walker_msgs::msg::StepStamped* step_l, double t){

        *step_r = r_tracker.predict_step(t);
        *step_l = l_tracker.predict_step(t);

    }




