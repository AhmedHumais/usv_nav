
#include <cstdio>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <algorithm>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float64.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"

#include <tf2_msgs/msg/tf_message.hpp>
#include <tf2/exceptions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include "pid.h"

using namespace std;

class UsvNavNode : public rclcpp::Node
{


private:
    void initNode();
    void loadParam();
    PID* pid_yaw;
    PID* pid_x;
    int mode=0;
    bool is_detected = false;
    bool target_loc_rcvd = false;
    bool curr_pos_rcvd = false;
    float current_heading = 0;
    float target_heading = 0;
    float target_err = 0;

    geometry_msgs::msg::Pose2D target_pos, current_pos, target_cord;
    rclcpp::Time last_detected, last_depth;

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr left_cmd_pub;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr right_cmd_pub;
    
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr target_err_sub;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr target_cord_sub;
    rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr target_sub;
    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr odom_sub;

    std::shared_ptr<tf2_ros::TransformListener> transform_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    rclcpp::TimerBase::SharedPtr timer_;


    void mainCallback();
    void target_err_callback(const std_msgs::msg::Float32 msg);
    void target_cord_callback(const geometry_msgs::msg::Point msg);
    void target_callback(const geometry_msgs::msg::Pose2D::SharedPtr msg);
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
    void odomCallback(const tf2_msgs::msg::TFMessage::SharedPtr msg);
    float calc_yaw_cmd(float yaw_error){
        return pid_yaw->calculate2(yaw_error);
    }
    float calc_x_cmd(float x_error){
        return pid_x->calculate2(x_error);
    }
    float compute_heading_error(float target_heading);
    void get_current_pos();

    void publish_cmd(float yaw_normalized, float thrust_normalized);
public:
    UsvNavNode();
    ~UsvNavNode(){};
};

