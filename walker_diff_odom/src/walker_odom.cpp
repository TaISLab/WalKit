
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

      // status vars
      first_reading_ = true;
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

      // ros objects
      RCLCPP_DEBUG(this->get_logger(), "Creating publishers/subscribers. ");
 
      transform_.header.frame_id = base_frame_id_;
      transform_.child_frame_id = odom_frame_id_;

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


void WalkerDiffDrive::leftEncoderCallback(const walker_msgs::msg::EncoderStamped::ConstSharedPtr left_enc_msg)  {
        // Left encoder
        int enc = left_enc_msg->encoder;
        if (enc<0){
            auto& clk = *this->get_clock();
            RCLCPP_ERROR_THROTTLE(this->get_logger(), clk, 500, "Error on left encoder [%d]. skipping value", enc);
            return;
        }
        //RCLCPP_INFO(this->get_logger(), "left encoder [%d]. ", enc);


        if ((enc < encoder_low_wrap_) && (prev_lencoder_ > encoder_high_wrap_)){
            lmult_ = lmult_ + 1;
        }

        if ((enc > encoder_high_wrap_) && (prev_lencoder_ < encoder_low_wrap_)){
            lmult_ = lmult_ - 1;
        }
            
        left_ = 1.0 * (enc + lmult_ * (encoder_max_ -encoder_min_));
        prev_lencoder_ = enc;
}

void WalkerDiffDrive::rightEncoderCallback(const walker_msgs::msg::EncoderStamped::ConstSharedPtr right_enc_msg)  {

        // Right encoder
        int enc = right_enc_msg->encoder;
        if (enc<0){
            auto& clk = *this->get_clock();
            RCLCPP_ERROR_THROTTLE(this->get_logger(), clk, 500, "Error on right encoder [%d]. skipping value", enc);
            return;
        }
        //RCLCPP_INFO(this->get_logger(), "right encoder [%d]. ", enc);


       if ((enc < encoder_low_wrap_) && (prev_rencoder_ > encoder_high_wrap_)){
            rmult_ = rmult_ + 1;
        }

        if ((enc > encoder_high_wrap_) && (prev_rencoder_ < encoder_low_wrap_)){
            rmult_ = rmult_ - 1;
        }
            
        right_ = 1.0 * (enc + rmult_ * (encoder_max_ -encoder_min_));
        prev_rencoder_ = enc;

    }

void WalkerDiffDrive::update(){

        now_ = this->get_clock()->now();
        rclcpp::Duration diff = now_ - then_;
        then_ = now_;
        double elapsed = diff.seconds();
            
        double d_left = 0;
        double d_right = 0;
        double d;
        double th;

        RCLCPP_INFO(this->get_logger(), "Updating. [%3.2f, %3.2f]",left_, right_);
 
        // calculate odometry
        if (first_reading_){
            first_reading_ = false;
        }else{
            d_left = (left_ - enc_left_) / ticks_meter_;
            d_right = (right_ - enc_right_) / ticks_meter_;
        }

        enc_left_ = left_;
        enc_right_ = right_;

        // traveled distance is the average of the wheels 
        d = (d_left + d_right) / 2.0;
        // this approximation works (in radians) for small angles
        th = (d_right - d_left) / base_width_;
        // calculate velocities
        dx_ = d / elapsed;
        dr_ = th / elapsed;

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

