#ifndef LEGSTRACKER_HH
#define LEGSTRACKER_HH


// Custom Messages related Headers
#include "walker_msgs/msg/step_stamped.hpp"
#include <walker_step_detector/track_leg.h>


    class LegsTracker{   

        public:
            LegsTracker();
            
            ~LegsTracker();

            void add_detection( geometry_msgs::msg::PointStamped position, float probability);

            void get_steps(walker_msgs::msg::StepStamped* step_r, walker_msgs::msg::StepStamped* step_l);

        private:
            TrackLeg l_tracker;
            TrackLeg r_tracker;


    };




#endif
