
#include "walker_diff_odom/walker_odom.hpp"

WalkerDiffDrive::WalkerDiffDrive() : Node("diff_tf") {     
      RCLCPP_DEBUG(this->get_logger(), "Loading params. ");
 
      // read params
      this->declare_parameter("tf_rate_hz", rclcpp::ParameterValue(10.0));
      this->get_parameter("tf_rate_hz", rate_hz_);
      this->declare_parameter("odom_topic_name", rclcpp::ParameterValue("odom"));
      this->get_parameter("odom_topic_name", odom_topic_name_);
      this->declare_parameter("left_wheel_encoder_topic_name", rclcpp::ParameterValue("left"));
      this->get_parameter("left_wheel_encoder_topic_name", left_wheel_encoder_topic_name_);
      this->declare_parameter("right_wheel_encoder_topic_name", rclcpp::ParameterValue("right"));
      this->get_parameter("right_wheel_encoder_topic_name", right_wheel_encoder_topic_name_);
      this->declare_parameter("ticks_meter", rclcpp::ParameterValue(50));
      this->get_parameter("ticks_meter", ticks_meter_ );
      this->declare_parameter("base_width", rclcpp::ParameterValue(0.245) );
      this->get_parameter("base_width", base_width_ );
      this->declare_parameter("base_frame_id", rclcpp::ParameterValue("base_link"));
      this->get_parameter("base_frame_id",  base_frame_id_ );
      this->declare_parameter("odom_frame_id", rclcpp::ParameterValue("odom"));
      this->get_parameter("odom_frame_id",  odom_frame_id_ );
      this->declare_parameter("encoder_min", rclcpp::ParameterValue(0));
      this->get_parameter("encoder_min",  encoder_min_ );
      this->declare_parameter("encoder_max", rclcpp::ParameterValue(4096) );
      this->get_parameter("encoder_max",  encoder_max_ );
      this->declare_parameter("wheel_low_wrap", rclcpp::ParameterValue(( encoder_max_ - encoder_min_)*3/10 + encoder_min_));
      this->get_parameter("wheel_low_wrap",  encoder_low_wrap_ );
      this->declare_parameter("wheel_high_wrap", rclcpp::ParameterValue(( encoder_max_ - encoder_min_)*7/10 + encoder_min_));
      this->get_parameter("wheel_high_wrap",  encoder_high_wrap_ );

      //
      RCLCPP_DEBUG(this->get_logger(), "CONFIGURATION .....................................");
      RCLCPP_DEBUG(this->get_logger(), "tf_rate_hz: [%3.3f] Hz.", rate_hz_);
      RCLCPP_DEBUG(this->get_logger(), "odom_topic_name: [%s]", odom_topic_name_.c_str());
      RCLCPP_DEBUG(this->get_logger(), "left_wheel_encoder_topic_name: [%s]", left_wheel_encoder_topic_name_.c_str());
      RCLCPP_DEBUG(this->get_logger(), "right_wheel_encoder_topic_name: [%s]", right_wheel_encoder_topic_name_.c_str());
      RCLCPP_DEBUG(this->get_logger(), "ticks_meter: [%5.4f] ticks", ticks_meter_ );
      RCLCPP_DEBUG(this->get_logger(), "base_width: [%3.3f] m", base_width_ );
      RCLCPP_DEBUG(this->get_logger(), "base_frame_id: [%s]",  base_frame_id_.c_str() );
      RCLCPP_DEBUG(this->get_logger(), "odom_frame_id: [%s]",  odom_frame_id_.c_str() );
      RCLCPP_DEBUG(this->get_logger(), "encoder_min: [%d] ticks",  encoder_min_ );
      RCLCPP_DEBUG(this->get_logger(), "encoder_max: [%d] ticks",  encoder_max_ );
      RCLCPP_DEBUG(this->get_logger(), "wheel_low_wrap: [%d] ticks",  encoder_low_wrap_ );
      RCLCPP_DEBUG(this->get_logger(), "wheel_high_wrap: [%d] ticks",  encoder_high_wrap_ );
      RCLCPP_DEBUG(this->get_logger(), ".....................................");

      // status vars
      first_reading_ = true;
      new_left_ = false;
      new_right_ = false;
      // encoder data
      enc_left_ = 0.0;
      enc_right_ = 0.0;
      //actual values coming back from robot
      left_ = 0.0; 
      right_ = 0.0;
      lmult_ = 0.0;
      rmult_ = 0.0;
      prev_lencoder_ = 0;
      prev_rencoder_ = 0;
      
      // position in xy plane
      x_ = 0.0;  
      y_ = 0.0;
      th_ = 0.0;
      
      // speeds in x/rotation
      dx_ = 0.0;  
      dr_ = 0.0;
      then_ = this->get_clock()->now();

      right_buffer_ = CircularBuffer(10);
      left_buffer_ = CircularBuffer(10);

      // ros objects
      RCLCPP_DEBUG(this->get_logger(), "Creating publishers/subscribers. ");
 
      transform_.header.frame_id = odom_frame_id_;
      transform_.child_frame_id = base_frame_id_;

      odom_.header.frame_id = odom_frame_id_;
      odom_.child_frame_id = base_frame_id_;

      
      odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>(odom_topic_name_, rate_hz_);
      // Initialize the transform broadcaster
      tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

      int period_ms = 1000.0 / rate_hz_;
      update_timer_ = this->create_wall_timer(std::chrono::milliseconds(period_ms), std::bind(&WalkerDiffDrive::update, this));

      //  inputs.
      sub_left_  = this->create_subscription<walker_msgs::msg::EncoderStamped>(left_wheel_encoder_topic_name_, 10, std::bind(&WalkerDiffDrive::leftEncoderCallback, this, _1));
      sub_right_ = this->create_subscription<walker_msgs::msg::EncoderStamped>(right_wheel_encoder_topic_name_, 10, std::bind(&WalkerDiffDrive::rightEncoderCallback, this, _1));

      RCLCPP_DEBUG(this->get_logger(), "Init done. ");
 
    }

/*

*/


void WalkerDiffDrive::leftEncoderCallback(const walker_msgs::msg::EncoderStamped::ConstSharedPtr left_enc_msg)  {
        // Left encoder
        int enc_raw = left_enc_msg->encoder;
        int enc;
        bool is_new_cycle = false;
        new_left_ = true;

        // get a usable value from encoder
        if (enc_raw<0){
            auto& clk = *this->get_clock();
            RCLCPP_ERROR_THROTTLE(this->get_logger(), clk, 500, "Error on left encoder [%d]. Using prev value", enc_raw);
            enc_raw = left_buffer_.getLast();
        }
        last_left_data_ = *left_enc_msg;      
        enc = enc_raw;

        // check if we are in a new turn
        if ((enc_raw < encoder_low_wrap_) && (prev_lencoder_ > encoder_high_wrap_)){
            lmult_ = lmult_ + 1;
            is_new_cycle = true;
            left_buffer_.clear();            
        }

        if ((enc_raw > encoder_high_wrap_) && (prev_lencoder_ < encoder_low_wrap_)){
            lmult_ = lmult_ - 1;
            is_new_cycle = true;
            left_buffer_.clear();
        }

        left_buffer_.add(enc_raw);
        if (!is_new_cycle){                        
            enc = left_buffer_.getAverage();
        } 
        
        //auto& clk = *this->get_clock();
        //RCLCPP_ERROR_THROTTLE(this->get_logger(), clk, 500, "LEFT: Raw [%d], Smooth [%d], loops [%3.0f]", enc_raw, enc, lmult_);

        left_ = 1.0 * (enc + lmult_ * (encoder_max_ -encoder_min_));
        prev_lencoder_ = enc;

        //auto& clk = *this->get_clock();
        //RCLCPP_ERROR_THROTTLE(this->get_logger(), clk, 500,  "left_ = enc + lmult_ * (encoder_max_ -encoder_min_) \n %3.0f = %d + %3.0f * %d ", left_, enc, lmult_, encoder_max_ - encoder_min_);

}

void WalkerDiffDrive::rightEncoderCallback(const walker_msgs::msg::EncoderStamped::ConstSharedPtr right_enc_msg)  {
        // Left encoder
        int enc_raw = right_enc_msg->encoder;
        int enc;
        bool is_new_cycle = false;
        new_right_ = true;

        // get a usable value from encoder
        if (enc_raw<0){
            auto& clk = *this->get_clock();
            RCLCPP_ERROR_THROTTLE(this->get_logger(), clk, 500, "Error on right encoder [%d]. Using prev value", enc_raw);
            enc_raw = right_buffer_.getLast();
        }
        last_right_data_ = *right_enc_msg;      
        enc = enc_raw;

        // check if we are in a new turn
        if ((enc_raw < encoder_low_wrap_) && (prev_rencoder_ > encoder_high_wrap_)){
            rmult_ = rmult_ + 1;
            is_new_cycle = true;
            right_buffer_.clear();            
        }

        if ((enc_raw > encoder_high_wrap_) && (prev_rencoder_ < encoder_low_wrap_)){
            rmult_ = rmult_ - 1;
            is_new_cycle = true;
            right_buffer_.clear();
        }

        right_buffer_.add(enc_raw);
        if (!is_new_cycle){                        
            enc = right_buffer_.getAverage();
        } 
        
        //auto& clk = *this->get_clock();
        //RCLCPP_ERROR_THROTTLE(this->get_logger(), clk, 500, "RIGHT: Raw [%d], Smooth [%d], loops [%3.0f]", enc_raw, enc, rmult_);

        right_ = -1.0 * ((enc)  + rmult_ * (encoder_max_ - encoder_min_)); // sign change is due to wheel being counterclock wise
        prev_rencoder_ = enc;

        //auto& clk = *this->get_clock();
        //RCLCPP_ERROR_THROTTLE(this->get_logger(), clk, 500,  "right_ = enc + rmult_ * (encoder_max_ -encoder_min_) \n %3.0f = %d + %3.0f * %d ", right_, enc, rmult_, encoder_max_ - encoder_min_);

    }

void WalkerDiffDrive::update(){

    if ((first_reading_) || ( new_right_) || (new_left_)) {
        new_right_ = false;
        new_left_ = false; 

        now_ = this->get_clock()->now();
        rclcpp::Duration diff = now_ - then_;
        then_ = now_;
        double elapsed = diff.seconds();
            
        double d_left = 0;
        double d_right = 0;
        double d;
        double th;
 
        //RCLCPP_ERROR(this->get_logger(), "\n\nUpdating. Total ticks L[%4.0f] - R[%4.0f]",left_, right_);

        // RCLCPP_ERROR(this->get_logger(),"Times L[%d s %d ns] - R[%d s %d ns] ", last_right_data_.header.stamp.sec, last_right_data_.header.stamp.nanosec, last_right_data_.header.stamp.sec, last_right_data_.header.stamp.nanosec  );
        // double time_diff   =  (last_right_data_.header.stamp.sec - last_left_data_.header.stamp.sec) * 1e9;
        // time_diff += (last_right_data_.header.stamp.nanosec - last_left_data_.header.stamp.nanosec) ;
        // time_diff = std::fabs(time_diff);
        // RCLCPP_ERROR(this->get_logger(), "Enc dif time [%3.3f] ns", time_diff );



        // calculate odometry
        if (first_reading_){
            first_reading_ = false;
        }else{
            d_left = (left_ - enc_left_) / ticks_meter_;
            d_right = (right_ - enc_right_) / ticks_meter_;
        }



        // RCLCPP_ERROR(this->get_logger(), "Dist increase L[%3.4f] - R[%3.4f] = [%3.4f]", d_left, d_right, d_left - d_right );

        enc_left_ = left_;
        enc_right_ = right_;

        // traveled distance is the average of the wheels 
        d = (d_left + d_right) / 2.0;

        // this approximation works (in radians) for small angles
        th = (d_right - d_left ) / base_width_;
        // calculate velocities
        dx_ = d / elapsed;
        dr_ = th / elapsed;

        //RCLCPP_ERROR(this->get_logger(),"travelled dist increase [%3.4f m, %3.4f deg]",d, th*180/3.1415);
 
        if (d != 0){
            // calculate distance traveled in x and y
            double x = cos(th) * d;
            double y = -sin(th) * d;
            // calculate the final position of the robot
            x_ = x_ + (cos(th_) * x - sin(th_) * y);
            y_ = y_ + (sin(th_) * x + cos(th_) * y);
        }
        if (th != 0){
            th_ = th_ + th;
        }


        //RCLCPP_ERROR(this->get_logger(), "New position [%3.4f, %3.4f, %3.4f deg]", x_, y_, th_*180/3.1415);

        // update transform info
        transform_.header.stamp = now_;        
        transform_.transform.translation.x = x_;
        transform_.transform.translation.y = y_;
        transform_.transform.translation.z = 0.0;
        tf2::Quaternion q;
        q.setRPY(0, 0, th_);
        transform_.transform.rotation.x = q.x();
        transform_.transform.rotation.y = q.y();
        transform_.transform.rotation.z = q.z();
        transform_.transform.rotation.w = q.w();

        // Send the transformation
        tf_broadcaster_->sendTransform(transform_);

        // And same with Odometry
        odom_.header.stamp = now_;
        odom_.pose.pose.position.x = x_;
        odom_.pose.pose.position.y = y_;
        odom_.pose.pose.position.z = 0.0;
        odom_.pose.pose.orientation.x = q.x();
        odom_.pose.pose.orientation.y = q.y();
        odom_.pose.pose.orientation.z = q.z();
        odom_.pose.pose.orientation.w = q.w();

        odom_.twist.twist.linear.x = dx_;
        odom_.twist.twist.linear.y = 0.0;
        odom_.twist.twist.angular.z = dr_;
        odom_publisher_->publish(odom_);
    }
}

