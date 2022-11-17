#include "usv_nav.hpp"
#include <iostream>

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(UsvNavNode, nodelet::Nodelet)



void UsvNavNode::onInit()
{
    ros::NodeHandle nh(getNodeHandle());
    ros::NodeHandle pnh(getPrivateNodeHandle());

    last_detected = ros::Time::now();
    last_depth = ros::Time::now();

    cmd_pub =  nh.advertise<std_msgs::Float32MultiArray>("/usv_cmd", 10);

    std::string imu_topic, odom_topic;
    // std::cout << "registering topics" << std::endl;
    pnh.param<std::string>("imu_topic", imu_topic, "/zedm/zed_node/imu/data");    
    pnh.param<std::string>("odom_topic", odom_topic, "/zedm/zed_node/odom");
    imu_sub = nh.subscribe<sensor_msgs::Imu>(imu_topic, 10, &UsvNavNode::imu_callback, this);
    target_err_sub = nh.subscribe<std_msgs::Float32>("/usv/target/error", 10, &UsvNavNode::target_err_callback, this);
    target_cord_sub = nh.subscribe<geometry_msgs::Point>("/usv/target/cord", 10, &UsvNavNode::target_cord_callback, this);
    target_sub = nh.subscribe<geometry_msgs::Pose2D>("target_vessel_pose", 10, &UsvNavNode::target_callback, this);
    #ifdef USE_ODOM
    odom_sub = nh.subscribe<nav_msgs::Odometry>(odom_topic, 10, &UsvNavNode::odomCallback, this);
    #endif
    pos_sub = nh.subscribe<geometry_msgs::Point>("/slam/pos", 10, &UsvNavNode::pos_callback, this);
    vel_sub = nh.subscribe<geometry_msgs::Point>("/slam/vel", 10, &UsvNavNode::vel_callback, this);
    ori_sub = nh.subscribe<geometry_msgs::Point>("/slam/ori", 10, &UsvNavNode::ori_callback, this);
    yaw_rate_sub = nh.subscribe<geometry_msgs::Point>("/navio/gyro", 10, &UsvNavNode::yaw_rate_callback, this);

    pd_sub = nh.subscribe<geometry_msgs::Point>("/set_pd", 2, &UsvNavNode::pd_callback, this);

    // std::cout << "registered" << std::endl;

    // tf_buffer_ =
    //     std::make_unique<tf2_ros::Buffer>(this->get_clock());
    // transform_listener_ =
    //     std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    
    pnh.param<double>("pid/kp_x", kp_x, 0.1);
    pnh.param<double>("pid/kd_x", kd_x, 10.0);
    pnh.param<double>("pid/kp_yaw", kp_yaw, 1.2);
    pnh.param<double>("pid/kd_yaw", kd_yaw, 12.0);

    pnh.param<bool>("tune_pid_x", tune_pid_x, false);
    pnh.param<bool>("tune_pid_yaw", tune_pid_yaw, false);

    std::cout << "loaded PID x Kp : " << kp_x << std::endl;

    pid_x = new PID(dt_, 1, -1, kp_x, kd_x, 0.0);
    pid_yaw = new PID(dt_, 1, -1, kp_yaw, kd_yaw, 0.0);

    timer_ = nh.createTimer(ros::Duration(dt_), &UsvNavNode::mainCallback, this);

}

void UsvNavNode::mainCallback(const ros::TimerEvent& event){
    // get_current_pos();
    if(mode ==0){
        if(curr_pos_rcvd){
            if(tune_pid_yaw){
                if(target_loc_rcvd){
                    auto yaw_err = compute_heading_error(target_pos.theta);
                    auto yaw_cmd = kp_yaw*yaw_err - kd_yaw*yaw_rate.z;
                    publish_cmd(yaw_cmd, 0);
                    return;
                }
            }
            if(tune_pid_x){
                if(target_loc_rcvd){
                    auto pos_err = target_pos.x - current_pos.x;
                    auto vel_err = -vel.x;
                    publish_cmd(0, pos_err*kp_x+kd_x*vel_err);
                    return;
                }
            }
            if(target_loc_rcvd){
                target_heading = atan2(target_pos.y - current_pos.y, target_pos.x - current_pos.x);
                mode =1;
            }
        }
        else{
            ROS_WARN("USV current position not received yet");
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
                publish_cmd(calc_yaw_cmd(heading_err), calc_x_cmd(sqrt(target_dist_sq)-0.5));
            }
        }
    }
    if(mode == 2){
        auto time_since_detected = ros::Time::now() - last_detected;
        auto time_since_depth = ros::Time::now() - last_depth;
        std::cout << "time_since_detected: " << time_since_detected.toSec() << std::endl;
        if(time_since_detected.toSec() > 0.4 || abs(target_err) > 0.35){
            is_detected = false;
            mode = 1;
            publish_cmd(0,0);
        }
        else if(time_since_depth.toSec() < 0.3 && target_cord.x <= 30) {
            mode = 3;
        }
        else{
            publish_cmd(calc_yaw_cmd(target_err), 0.8);
        }
    }
    if(mode == 3){
        auto time_since_update = ros::Time::now() - last_depth;
        if(time_since_update.toSec() > 0.4){
            mode = 2;
            publish_cmd(0,0);
        }else{
            publish_cmd(calc_yaw_cmd(atan2(target_cord.y, target_cord.x)), calc_x_cmd(target_cord.x - 0.8));
            // if(target_cord.x <= 10){
            //     publish_cmd(calc_yaw_cmd(atan2(target_cord.y, target_cord.x)), 0);
            // }
        }
    }
    std::cout << "current mode : " << mode << std::endl;

}

void UsvNavNode::imu_callback(const sensor_msgs::Imu::ConstPtr &msg){
    // auto ori = msg->orientation;
    // tf2::Quaternion q(ori.x, ori.y, ori.z, ori.w);
    // tf2::Matrix3x3 R_(q);
    // tf2Scalar x, y, z;
    // R_.getEulerYPR(z, y, x);
    // current_heading = z < 0? z+2*M_PI : z;
}

void UsvNavNode::target_callback(const geometry_msgs::Pose2D msg){
    target_loc_rcvd = true;
    target_pos = msg;
    target_pos.theta = current_heading + 0.5; // for tuning
}

void UsvNavNode::target_err_callback(const std_msgs::Float32 msg){
    target_err = msg.data;
    last_detected = ros::Time::now();
    is_detected = true;
}

void UsvNavNode::target_cord_callback(const geometry_msgs::Point msg){
    last_depth = ros::Time::now();
    target_cord.x = msg.x;
    target_cord.y = msg.y;
}

void UsvNavNode::publish_cmd(float yaw_normalized, float thrust_normalized){
    if(yaw_normalized < -1.0){
        yaw_normalized = -1.0;
    }
    if(yaw_normalized > 1.0){
        yaw_normalized = 1.0;
    }
    if(thrust_normalized < -1.0){
        thrust_normalized = -1.0;
    }
    if(thrust_normalized > 1.0){
        thrust_normalized = 1.0;
    }
    // float left_cmd = thrust_normalized-yaw_normalized;
    // float right_cmd = thrust_normalized+yaw_normalized;
    
    // float high_cmd = left_cmd > right_cmd? left_cmd : right_cmd;
    // float low_cmd = left_cmd < right_cmd? left_cmd : right_cmd;
    // float offset = 0;
    // if(high_cmd > 1.0){
    //     std::cout<< "offset_cmd : " << offset << std::endl;
    //     offset = high_cmd-1;
    // }
    // if(low_cmd < -1){
    //     offset = -(low_cmd+1);
    // }
    // left_cmd -= offset; right_cmd -= offset;

    // if(abs(left_cmd) > 1.0) { left_cmd /= abs(left_cmd);}
    // if(abs(right_cmd) > 1.0) { right_cmd /= abs(right_cmd);}

    std_msgs::Float32MultiArray cmd;
    cmd.data.push_back(thrust_normalized);
    cmd.data.push_back(yaw_normalized);
    cmd_pub.publish(cmd);
}
#ifdef USE_ODOM
void UsvNavNode::odomCallback(const nav_msgs::Odometry::ConstPtr &msg){
    if(!curr_pos_rcvd){
        std::cout << "Odom callback called" <<std::endl;
    }
    // auto tfs = msg->transforms;
    // bool found_tf = false;
    // for(auto tf : tfs){
    //     if(tf.header.frame_id.compare("coast")==0 && tf.child_frame_id.compare("usv") == 0){
    //         std::cout << "frame id " << tf.header.frame_id << "child_frame_id" << tf.child_frame_id << std::endl;
    current_pos.x = msg->pose.pose.position.x;
    current_pos.y = msg->pose.pose.position.y;

    auto q = msg->pose.pose.orientation;
    double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    current_heading = std::atan2(siny_cosp, cosy_cosp);
 
            // found_tf = true;
    curr_pos_rcvd = true;
    //     }
    // }
    // if(!found_tf){
    //     RCLCPP_ERROR(this->get_logger(), "ground truth position of usv not available");
    // }
}
#endif

void UsvNavNode::get_current_pos(){
    // geometry_msgs::msg::TransformStamped transformStamped;

    // // Look up for the transformation between target_frame and turtle2 frames
    // // and send velocity commands for turtle2 to reach target_frame
    // std::string to_frame = "odom";
    // std::string from_frame = "usv";
    // try {
    //     transformStamped = tf_buffer_->lookupTransform(
    //     to_frame, from_frame,
    //     tf2::TimePointZero);
    // } catch (tf2::TransformException & ex) {
    //     RCLCPP_ERROR(this->get_logger(), "ground truth position of usv not available");
    //     return;
    // }
    // curr_pos_rcvd = true;
    // current_pos.x = transformStamped.transform.translation.x;
    // current_pos.y = transformStamped.transform.translation.y;
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

void UsvNavNode::pos_callback(const geometry_msgs::Point::ConstPtr &msg){
    if(!curr_pos_rcvd){
        std::cout << "Position Received" <<std::endl;
    }
    curr_pos_rcvd = true;

    current_pos.x = msg->x;
    current_pos.y = msg->y;
    pos = (*msg);
}

void UsvNavNode::vel_callback(const geometry_msgs::Point::ConstPtr &msg){
    vel = (*msg);
}

void UsvNavNode::ori_callback(const geometry_msgs::Point::ConstPtr &msg){
    ori = (*msg);
    current_heading = msg->z;
}

void UsvNavNode::yaw_rate_callback(const geometry_msgs::Point::ConstPtr &msg){
    yaw_rate = (*msg);
}

void UsvNavNode::pd_callback(const geometry_msgs::Point::ConstPtr &msg){
    if(tune_pid_yaw){
        kp_yaw = msg->x;
        kd_yaw = msg->y;
    }
    else if(tune_pid_x){
        kp_x = msg->x;
        kd_x = msg->y;
    }
}