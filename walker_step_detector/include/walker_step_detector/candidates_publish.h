#ifndef CANDIDATES_PUBLISH_HH
#define CANDIDATES_PUBLISH_HH

// Conversion compartida por los tres detectores (detect_steps, km_detect_steps,
// detect_steps_s) para publicar su lista de candidatos CRUDOS (antes de entrar
// a su propio LegsTracker) como walker_msgs/StepArray -- consumido por
// detect_steps_fused para fusionar los tres front-ends en un unico tracker.
// StepArray ya existia en walker_msgs sin usarse en ningun sitio.

#include <list>

#include "std_msgs/msg/header.hpp"
#include "walker_msgs/msg/step.hpp"
#include "walker_msgs/msg/step_array.hpp"
#include "walker_msgs/msg/step_stamped.hpp"

namespace walker_step_detector
{

inline walker_msgs::msg::StepArray to_step_array(
    const std_msgs::msg::Header & header,
    const std::list<walker_msgs::msg::StepStamped> & points)
{
    walker_msgs::msg::StepArray arr;
    arr.header = header;
    for (const auto & p : points) {
        walker_msgs::msg::Step s;
        s.position = p.position.point;
        s.confidence = p.confidence;
        s.tracked = p.tracked;
        s.speed = p.speed;
        s.load = p.load;
        arr.steps.push_back(s);
    }
    return arr;
}

}  // namespace walker_step_detector

#endif  // CANDIDATES_PUBLISH_HH
