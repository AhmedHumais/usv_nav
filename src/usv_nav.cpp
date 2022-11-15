#include "usv_nav.hpp"
#include <iostream>

using namespace std::chrono_literals;

UsvNavNode::UsvNavNode()
    : Node("usv_nav_node")
{using namespace std::chrono_literals;

  initNode();
  
}

void UsvNavNode::loadParam()
{

}

void UsvNavNode::initNode()
{
    last_detected = this->now();
    last_depth = this->now();

    left_cmd_pub =  this->create_publisher<std_msgs::msg::Float64>(
    "/usv/left/thrust/cmd_thrust", 10);
    right_cmd_pub =  this->create_publisher<std_msgs::msg::Float64>(
    "/usv/right/thrust/cmd_thrust", 10);

    imu_sub = this->create_subscription<sensor_msgs::msg::Imu>("/usv/imu/data", 10, 
        std::bind(&UsvNavNode::imu_callback, this, std::placeholders::_1));
    target_err_sub = this->create_subscription<std_msgs::msg::Float32>("/usv/target/error", 10,
        std::bind(&UsvNavNode::target_err_callback, this, std::placeholders::_1));
    target_cord_sub = this->create_subscription<geometry_msgs::msg::Point>("/usv/target/cord", 10,
        std::bind(&UsvNavNode::target_cord_callback, this, std::placeholders::_1));
    target_sub = this->create_subscription<geometry_msgs::msg::Pose2D>("target_vessel_pose", 10,
        std::bind(&UsvNavNode::target_callback, this, std::placeholders::_1));
    odom_sub = this->create_subscription<tf2_msgs::msg::TFMessage>("/usv/pose_static", 10,
        std::bind(&UsvNavNode::odomCallback, this, std::placeholders::_1));

    tf_buffer_ =
        std::make_unique<tf2_ros::Buffer>(this->get_clock());
    transform_listener_ =
        std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    pid_yaw = new PID(0.05, 1, -1, 1.2, 12.0, 0.0);
    pid_x = new PID(0.05, 1, -1, 0.1, 10.0, 0.0);

    timer_ = create_wall_timer(
      50ms, std::bind(&UsvNavNode::mainCallback, this));

}

void UsvNavNode::mainCallback(){
    // get_current_pos();
    if(mode ==0){
        if(curr_pos_rcvd){
            if(target_loc_rcvd){
                target_heading = atan2(target_pos.y - current_pos.y, target_pos.x - current_pos.x);
                mode =1;
            }
        }
        else{
            RCLCPP_WARN(this->get_logger(), "USV current position not received yet");
        }
    }
    if(mode ==1){
        auto heading_err = compute_heading_error(target_heading);
        std::cout << "target pos: " << target_pos.x << ", " << target_pos.y << std::endl;
        std::cout << "current pos: " << current_pos.x << ", " << current_pos.y << std::endl;

        std::cout << "current_heading: " << current_heading << ", " << "target_heading: " << target_heading << ", " << "heading_err: " << heading_err << std::endl;

        if(abs(heading_err) > 0.175){
            auto yaw_cmd = calc_yaw_cmd(heading_err);
            std::cout << "yaw_cmd : " << yaw_cmd <<std::endl;
            publish_cmd(yaw_cmd, 0);
        }
        else{
            if(is_detected){
                mode = 2;
            }else{
                auto target_dist_sq = (target_pos.x-current_pos.x)*(target_pos.x-current_pos.x) + (target_pos.y-current_pos.y)*(target_pos.y-current_pos.y);
                publish_cmd(calc_yaw_cmd(heading_err), calc_x_cmd(sqrt(target_dist_sq)-15));
            }
        }
    }
    if(mode == 2){
        auto time_since_detected = this->now() - last_detected;
        auto time_since_depth = this->now() - last_depth;
        std::cout << "time_since_detected: " << time_since_detected.seconds() << std::endl;
        if(time_since_detected.seconds() > 0.4 || abs(target_err) > 0.35){
            is_detected = false;
            mode = 1;
            publish_cmd(0,0);
        }
        else if(time_since_depth.seconds() < 0.3 && target_cord.x <= 30) {
            mode = 3;
        }
        else{
            publish_cmd(calc_yaw_cmd(target_err), 0.9);
        }
    }
    if(mode == 3){
        auto time_since_update = this->now() - last_depth;
        if(time_since_update.seconds() > 0.4){
            mode = 2;
            publish_cmd(0,0);
        }else{
            publish_cmd(calc_yaw_cmd(atan2(target_cord.y, target_cord.x)), calc_x_cmd(target_cord.x - 16));
            if(target_cord.x <= 10){
                publish_cmd(calc_yaw_cmd(atan2(target_cord.y, target_cord.x)), 0);
            }
        }
    }
    std::cout << "current mode : " << mode << std::endl;

}

void UsvNavNode::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg){
    auto ori = msg->orientation;
    tf2::Quaternion q(ori.x, ori.y, ori.z, ori.w);
    tf2::Matrix3x3 R_(q);
    tf2Scalar x, y, z;
    R_.getEulerYPR(z, y, x);
    current_heading = z < 0? z+2*M_PI : z;
}

void UsvNavNode::target_callback(const geometry_msgs::msg::Pose2D::SharedPtr msg){
    target_loc_rcvd = true;
    target_pos = *msg;
}

void UsvNavNode::target_err_callback(const std_msgs::msg::Float32 msg){
    target_err = msg.data;
    last_detected = this->now();
    is_detected = true;
}

void UsvNavNode::target_cord_callback(const geometry_msgs::msg::Point msg){
    last_depth = this->now();
    target_cord.x = msg.x;
    target_cord.y = msg.y;
}

void UsvNavNode::publish_cmd(float yaw_normalized, float thrust_normalized){
    float left_cmd = thrust_normalized-yaw_normalized;
    float right_cmd = thrust_normalized+yaw_normalized;
    
    float high_cmd = left_cmd > right_cmd? left_cmd : right_cmd;
    float low_cmd = left_cmd < right_cmd? left_cmd : right_cmd;
    float offset = 0;
    if(high_cmd > 1.0){
        std::cout<< "offset_cmd : " << offset << std::endl;
        offset = high_cmd-1;
    }
    if(low_cmd < -1){
        offset = -(low_cmd+1);
    }
    left_cmd -= offset; right_cmd -= offset;

    if(abs(left_cmd) > 1.0) { left_cmd /= abs(left_cmd);}
    if(abs(right_cmd) > 1.0) { right_cmd /= abs(right_cmd);}

    std_msgs::msg::Float64 cmd;
    cmd.data = left_cmd*100;
    left_cmd_pub->publish(cmd);
    cmd.data = right_cmd*100;
    right_cmd_pub->publish(cmd);

}

void UsvNavNode::odomCallback(const tf2_msgs::msg::TFMessage::SharedPtr msg){
    std::cout << "Odom callback called" <<std::endl;
    auto tfs = msg->transforms;
    bool found_tf = false;
    for(auto tf : tfs){
        if(tf.header.frame_id.compare("coast")==0 && tf.child_frame_id.compare("usv") == 0){
            std::cout << "frame id " << tf.header.frame_id << "child_frame_id" << tf.child_frame_id << std::endl;
            current_pos.x = tf.transform.translation.x;
            current_pos.y = tf.transform.translation.y;
            found_tf = true;
            curr_pos_rcvd = true;
        }
    }
    if(!found_tf){
        RCLCPP_ERROR(this->get_logger(), "ground truth position of usv not available");
    }
}

void UsvNavNode::get_current_pos(){
    geometry_msgs::msg::TransformStamped transformStamped;

    // Look up for the transformation between target_frame and turtle2 frames
    // and send velocity commands for turtle2 to reach target_frame
    std::string to_frame = "odom";
    std::string from_frame = "usv";
    try {
        transformStamped = tf_buffer_->lookupTransform(
        to_frame, from_frame,
        tf2::TimePointZero);
    } catch (tf2::TransformException & ex) {
        RCLCPP_ERROR(this->get_logger(), "ground truth position of usv not available");
        return;
    }
    curr_pos_rcvd = true;
    current_pos.x = transformStamped.transform.translation.x;
    current_pos.y = transformStamped.transform.translation.y;
}

float UsvNavNode::compute_heading_error(float target_heading){
    if(target_heading < 0){
        target_heading += 2*M_PI;
    }
    auto h_err = target_heading - current_heading;
    if(h_err > M_PI){
        h_err -= 2*M_PI;
    }

    return h_err;
}