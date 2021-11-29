#ifndef USBCONN_HPP_
#define USBCONN_HPP_

#include <chrono>
#include <functional>
#include <memory>
#include <libserial/SerialStream.h>
#include <libserial/SerialStream.h>

#include <cstdlib>
#include <iomanip>
#include <unistd.h>

#include <filesystem>
#include <string>       // std::string
#include <iostream>     // std::cout
#include <sstream>      // std::stringstream

#include "rclcpp/rclcpp.hpp"
#include "walker_msgs/msg/encoder_stamped.hpp"
#include "walker_msgs/msg/force_stamped.hpp"

using namespace std::chrono_literals;
using namespace LibSerial ;

class HandlePublisher : public rclcpp::Node
{
  public:
    // topic where data is published
    rclcpp::Publisher<walker_msgs::msg::EncoderStamped>::SharedPtr encoder_publisher_; 
    rclcpp::Publisher<walker_msgs::msg::ForceStamped>::SharedPtr force_publisher_; 

    // serial usb where data is coming from
    SerialStream serial_stream_;
    
    // port descriptor base name ( ttyACM, USB, ...)
    std::string basename_;

    // name of the publishing topic
    std::string topic_name_;

    // full port name e.g. /dev/ttyUSB1 
    std::string port_name_;

    // string at the beginning of every msg
    std::string preamble_;

    // frame id describing where data was published 
    std::string frame_id_;

    std::string data_type_;

    // char ending each data packet
    char endMarker_ ;

    HandlePublisher();
    ~HandlePublisher();
    void start();

    std::string get_serial_port(std::string preamble, int highest_port) ;

}; // END HandlePublisher Class

#endif // USBCONN_HPP_