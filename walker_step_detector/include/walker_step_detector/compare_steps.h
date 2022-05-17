#ifndef COMPARESTEPS_HH
#define COMPARESTEPS_HH


// Custom Messages related Headers
#include "walker_msgs/msg/step_stamped.hpp"
#include <math.h>

class CompareSteps{

    public:
        CompareSteps();
        
    //    CompareSteps(walker_msgs::msg::StepStamped origin);
        
        bool operator ()(const walker_msgs::msg::StepStamped & a, const walker_msgs::msg::StepStamped & b);

        static float dist(walker_msgs::msg::StepStamped a, walker_msgs::msg::StepStamped o);

    //private:
    //    walker_msgs::msg::StepStamped orig_;

};


#endif //COMPARESTEPS_HH
