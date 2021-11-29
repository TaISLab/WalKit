
#include "walker_arduino/USBConn.hpp"

HandlePublisher::HandlePublisher() : Node("handle_publisher"){
        /*
        TODO: I got some problems using arguments and params with ':' or '\n' symbols,
        so I'm hardcoding them here.
        */
        
        this->declare_parameter<std::string>( "serial_preamble",  "RL" ); 
        this->get_parameter("serial_preamble", preamble_);
        preamble_ = preamble_ + ":";
        endMarker_ = '\n';
 
        this->declare_parameter<std::string>( "handle_topic_name", "left_handle" ); 
        this->get_parameter("handle_topic_name", topic_name_);

        this->declare_parameter<std::string>( "data_type", "float" ); 
        this->get_parameter("data_type", data_type_);

        this->declare_parameter<std::string>( "handle_frame_id",  "left_handle_id" ); 
        this->get_parameter("handle_frame_id", frame_id_);

        this->declare_parameter<std::string>( "port_basename",  "/dev/ttyUSB") ; 
        this->get_parameter("port_basename", basename_ );
        
        if (data_type_.compare("int")==0) {
            encoder_publisher_ = this->create_publisher<walker_msgs::msg::EncoderStamped>(topic_name_, 10);
        } else if (data_type_.compare("float")==0) {
            force_publisher_ = this->create_publisher<walker_msgs::msg::ForceStamped>(topic_name_, 10);
        } else{
            RCLCPP_FATAL(this->get_logger(), "'Data Type [%s] is nor int or float ", data_type_.c_str() ); 
        }
        
        port_name_ = this->get_serial_port(preamble_, 8); 
        
        try{
            // Open the Serial Port at the desired hardware port.
            serial_port_.Open(port_name_) ;
        }    catch (const OpenFailed&) {
            RCLCPP_FATAL(this->get_logger(), "The serial port [%s] did not open correctly", port_name_.c_str() ); 
        }

        // Set the baud rate of the serial port.
        serial_port_.SetBaudRate(BaudRate::BAUD_115200) ;
        // Set the number of data bits.
        serial_port_.SetCharacterSize(CharacterSize::CHAR_SIZE_8) ;
        // Turn off hardware flow control.
        serial_port_.SetFlowControl(FlowControl::FLOW_CONTROL_NONE) ;
        // Disable parity.
        serial_port_.SetParity(Parity::PARITY_NONE) ;
        // Set the number of stop bits.
        serial_port_.SetStopBits(StopBits::STOP_BITS_1) ;

}

HandlePublisher::~HandlePublisher(){
    if (serial_port_.IsOpen()) {
        serial_port_.Close();
    }
}

void HandlePublisher::start(){
        bool isRecvStarted = false;
        char rc;
        int timeout_ms = 25; //in milliseconds
 
        std::stringstream ss;
        std::string data;
        
        walker_msgs::msg::ForceStamped f_measurement;
        walker_msgs::msg::EncoderStamped e_measurement;
    
        f_measurement.header.frame_id = e_measurement.header.frame_id = frame_id_;
        RCLCPP_INFO(this->get_logger(), "Publishing handle measurements from port [%s] at topic [%s]", port_name_.c_str(), topic_name_.c_str());

        while (rclcpp::ok()) {
            try{
                serial_port_.ReadByte( rc ,timeout_ms);
            }catch (const ReadTimeout&){
                RCLCPP_ERROR(this->get_logger(), "Reading char from port [%s] didn't succeed after [%d] ms. Something is odd...", port_name_.c_str(), timeout_ms);                
            }

            if (isRecvStarted == true) {
                if (rc != endMarker_) {
                    ss << rc;
                } else {
                    data = ss.str();
                    f_measurement.header.stamp = e_measurement.header.stamp = this->get_clock()->now();
                    // remove preamble and cast data to number:
                    
                    if (data_type_.compare("float")==0) {
                        f_measurement.force = std::stof(data.substr(preamble_.length()));
                        RCLCPP_DEBUG(this->get_logger(), "data is: [%3.2f]", f_measurement.force);
                    } else if (data_type_.compare("int")==0) {
                        e_measurement.encoder = std::stoi(data.substr(preamble_.length()));
                        RCLCPP_DEBUG(this->get_logger(), "data is: [%d]", e_measurement.encoder);
                    }

                    // send data
                    try {
                        if (data_type_.compare("float")==0) {
                            force_publisher_->publish(f_measurement);
                        } else if (data_type_.compare("int")==0) {
                            encoder_publisher_->publish(e_measurement);
                        }

                    } catch (const rclcpp::exceptions::RCLError &e) {
                        RCLCPP_ERROR( this->get_logger(), "unexpectedly failed with %s", e.what());
                    }

                    // clear stream and carry on
                    ss.str(std::string());

                }
            }

            else if (rc == endMarker_) {
                isRecvStarted = true;
            }

            try {
                rclcpp::spin_some(this->get_node_base_interface());
            } catch (const rclcpp::exceptions::RCLError &e) {
                RCLCPP_ERROR( this->get_logger(), "unexpectedly failed with %s", e.what());
            }

        }
        serial_port_.Close();

}

std::string HandlePublisher::get_serial_port(std::string preamble, int highest_port) {
        std::string port_name;
        std::stringstream name_builder;
        SerialPort test_serial_port;
        bool isSearchStarted = false;
        bool isSearchFinished = false;
        std::stringstream sb;
        std::string data;
        char rc;
        int timeout_ms = 25; //in milliseconds

        int attempts = 0;
        int max_attempts = 5;

        for (int i=0;i<=highest_port;i++){
            // build a port name
            // this adds padding to the number
            //name_builder << basename << std::setw(2) << std::setfill('0') << i;
            name_builder << basename_  << i;
            port_name = name_builder.str();
            name_builder.str(std::string());

            // open it
            try {
                // Open the Serial Port at the desired hardware port.
                RCLCPP_DEBUG(this->get_logger(), "Opening serial port [%s].", port_name.c_str());
                test_serial_port.Open( port_name );
            } catch (const OpenFailed&) {
                RCLCPP_ERROR(this->get_logger(), "The serial port [%s] couldn't be opened. Trying next", port_name.c_str());
                test_serial_port.Close(  );
                continue;
            }
            RCLCPP_DEBUG(this->get_logger(), "Serial port [%s] opened", port_name.c_str());
            
            // Set the baud rate of the serial port.
            test_serial_port.SetBaudRate(BaudRate::BAUD_115200) ;
            // Set the number of data bits.
            test_serial_port.SetCharacterSize(CharacterSize::CHAR_SIZE_8) ;
            // Turn off hardware flow control.
            test_serial_port.SetFlowControl(FlowControl::FLOW_CONTROL_NONE) ;
            // Disable parity.
            test_serial_port.SetParity(Parity::PARITY_NONE) ;
            // Set the number of stop bits.
            test_serial_port.SetStopBits(StopBits::STOP_BITS_1) ;


            // Wait for data to be available at the serial port.
            while( (test_serial_port.IsDataAvailable()) && (attempts<max_attempts) ){
                usleep(1000);
                RCLCPP_DEBUG(this->get_logger(), "Waiting for data at [%s].", port_name.c_str());
                attempts ++;
            }

            if (attempts<max_attempts) {
                RCLCPP_DEBUG(this->get_logger(), "The serial port [%s] didn't show data after a while... skipping.", port_name.c_str());
                test_serial_port.Close();
                continue;
            }

            // check if streamed data at port starts with the right preamble
            isSearchStarted = false;
            sb.str(std::string());
            RCLCPP_DEBUG(this->get_logger(), "Looking for preamble [%s] ", preamble.c_str());
            isSearchFinished = false;
            int num_chars =0;
            while (!isSearchFinished) {
                
                try{
                    test_serial_port.ReadByte( rc,timeout_ms );
                   }catch (const ReadTimeout&){
                    RCLCPP_DEBUG(this->get_logger(), "Reading char from port [%s] didn't succeed after [%d] ms. Skipping", port_name.c_str(), timeout_ms);                
                    test_serial_port.Close();
                    isSearchFinished = true;                        
                }
               
                // search for preamble starts AFTER endMarker
                if (isSearchStarted == true) {
                    sb << rc;
                    data = sb.str();
                    RCLCPP_DEBUG(this->get_logger(), "last char is: [%c]", rc );
                    if (data.compare(preamble_)==0) {
                        //port found!
                        RCLCPP_DEBUG(this->get_logger(), "Port found!. [%s] is using preamble [%s]", port_name.c_str(), preamble.c_str());
                        test_serial_port.Close();
                        return port_name;
                    }
                    if (rc == endMarker_) {
                        RCLCPP_DEBUG(this->get_logger(), "The serial port [%s] didn't have preamble [%s] but [%s]", port_name.c_str(), preamble_.c_str(), data.substr(0,preamble_.length()).c_str());
                        test_serial_port.Close();
                        isSearchFinished = true;                        
                    }

                } else if (rc == endMarker_) {
                    RCLCPP_DEBUG(this->get_logger(), "End marker detected. ");
                    if (!isSearchStarted){ 
                        isSearchStarted = true;
                        RCLCPP_DEBUG(this->get_logger(), "Starting preamble search");
                    } else{
                        RCLCPP_DEBUG(this->get_logger(), "The serial port [%s] didn't have preamble [%s] but [%s]", port_name.c_str(), preamble_.c_str(), data.c_str());
                        test_serial_port.Close();
                        isSearchFinished = true; 
                    }
                }
                if (num_chars > 20){
                    RCLCPP_DEBUG(this->get_logger(), "Couldn't find preamble [%s] in port [%s] after reading [%d] chars", preamble_.c_str(),port_name.c_str(),num_chars);                
                    test_serial_port.Close();
                    isSearchFinished = true;                        
                } else {
                    num_chars++;
                    RCLCPP_DEBUG(this->get_logger(), "[%d] chars read in port [%s]", num_chars,port_name.c_str());                
                        
                }
            
            } 

        }
        RCLCPP_ERROR(this->get_logger(), "Couldn't find preamble [%s] in ports [%s0] to [%s%i]", preamble_.c_str(),basename_.c_str(),basename_.c_str(),highest_port);                
        return "";
}

