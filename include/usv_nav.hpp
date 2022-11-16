
#include <cstdio>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <algorithm>
#include <cmath>

#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/Imu.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float32MultiArray.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Pose2D.h"
#include "nav_msgs/Odometry.h"

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <nodelet/nodelet.h>

#include "pid.h"

using namespace std;

const int FREQ = 20;

class UsvNavNode : public nodelet::Nodelet
{

private:
    virtual void onInit();
    PID* pid_yaw;
    PID* pid_x;
    double dt_ = 1.0/FREQ;
    int mode=0;
    bool is_detected = false;
    bool target_loc_rcvd = false;
    bool curr_pos_rcvd = false;
    float current_heading = 0;
    float target_heading = 0;
    float target_err = 0;

    geometry_msgs::Pose2D target_pos, current_pos, target_cord;
    ros::Time last_detected, last_depth;

    ros::Publisher cmd_pub;

    
    ros::Subscriber imu_sub;
    ros::Subscriber target_err_sub;
    ros::Subscriber target_cord_sub;
    ros::Subscriber target_sub;
    ros::Subscriber odom_sub;

    std::shared_ptr<tf2_ros::TransformListener> transform_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    ros::Timer timer_;

    void mainCallback(const ros::TimerEvent& event);
    void target_err_callback(const std_msgs::Float32 msg);
    void target_cord_callback(const geometry_msgs::Point msg);
    void target_callback(const geometry_msgs::Pose2D msg);
    void imu_callback(const sensor_msgs::Imu::ConstPtr &msg);
    void odomCallback(const nav_msgs::Odometry::ConstPtr &msg);
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
    UsvNavNode() = default;
    ~UsvNavNode(){};
};

