#ifndef AGV_ODOM_H
#define AGV_ODOM_H

#include <ros/ros.h>
#include <ros/time.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <boost/asio.hpp>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <omh_robot_base/MotorState.h>


namespace agv
{

#define GEAR_RATIO 16
#define MY_PI 3.141592653

class agv
{
public:
    agv();
    ~agv();
    bool init();
    bool spinOnce();
   
private:
    bool readSpeed();
    void laser_callback(const sensor_msgs::LaserScanConstPtr &msg);
    void motorstate_callback(const omh_robot_base::MotorState & msg);
   
private:
    ros::Time current_time_, last_time_;

    double x_;// x 方向位移
    double y_;// y 方向位移
    double th_;// z 方向角度

    double vx_;// x 方向线速度 m/s
    double vy_;
    double vth_;// 角速度 rad / s

    double vel_left_;// agv 左轮线速度 m / s
    double vel_right_;// agv 右轮线速度 m / s

    double omega_left_;// agv 左轮角速度 rad/s
    double omega_right_;// agv 右轮角速度 rad/s

    std::string scan_topic;
    std::string motor_state_topic;
    std::string odom_topic;

    double left_wheel_radius_;
    double right_wheel_radius_;
    double wheel_base_;

    ros::NodeHandle nh;
    ros::Publisher pub_;
    ros::Subscriber raw_laser_sub_;
    ros::Subscriber raw_odom_sub_;
    tf::TransformBroadcaster odom_broadcaster_;
};
    
}

#endif /* AGV_ODOM_H */
