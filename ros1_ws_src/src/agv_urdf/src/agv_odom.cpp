#include <vector>
#include "agv_urdf/agv_odom.h"


namespace agv
{
boost::array<double, 36> odom_pose_covariance = {
    {1e-9, 0, 0, 0, 0, 0, 
    0, 1e-3, 1e-9, 0, 0, 0, 
    0, 0, 1e6, 0, 0, 0,
    0, 0, 0, 1e6, 0, 0, 
    0, 0, 0, 0, 1e6, 0, 
    0, 0, 0, 0, 0, 1e-9}};
boost::array<double, 36> odom_twist_covariance = {
    {1e-9, 0, 0, 0, 0, 0, 
    0, 1e-3, 1e-9, 0, 0, 0, 
    0, 0, 1e6, 0, 0, 0, 
    0, 0, 0, 1e6, 0, 0, 
    0, 0, 0, 0, 1e6, 0, 
    0, 0, 0, 0, 0, 1e-9}};


agv::agv():
    x_(0.0), y_(0.0), th_(0.0),
    vx_(0.0), vy_(0.0), vth_(0.0),
    vel_left_(0.0), vel_right_(0.0),
    left_wheel_radius_(0.0), right_wheel_radius_(0.0), wheel_base_(0.0), nh("~")
{
    nh.param<std::string>("scan_topic_sub", scan_topic, "/scan");
    nh.param<std::string>("motor_state_topic_sub", motor_state_topic, "/motor_state");
    nh.param<std::string>("odom_topic_pub", odom_topic, "/odom");
    // nh.param<double>("left_wheel_radius", left_wheel_radius_, 0.0939013);
    // nh.param<double>("right_wheel_radius", right_wheel_radius_, 0.0935378);
    nh.param<double>("left_wheel_radius", left_wheel_radius_, 0.09);
    nh.param<double>("right_wheel_radius", right_wheel_radius_, 0.09);
    nh.param<double>("wheel_base", wheel_base_, 0.544804);
}

agv::~agv()
{
}

bool agv::init()
{
    ros::Time::init();
	current_time_ = ros::Time::now();
	last_time_ = ros::Time::now();
	
    //定义发布消息的名称
    pub_ = nh.advertise<nav_msgs::Odometry>(odom_topic, 1000, true);
    // raw_laser_sub_ = nh.subscribe(scan_topic, 100, &agv::laser_callback, this);
    raw_odom_sub_ = nh.subscribe(motor_state_topic, 1000, &agv::motorstate_callback, this);
    
    return true;
}

bool agv::readSpeed()
{
    // 积分计算里程计信息
    vx_  = (vel_right_ + vel_left_) / 2;// x 方向线速度 m/s
    vth_ = (vel_right_ - vel_left_) / wheel_base_;// 角速度 rad / s
    
    ros::Time curr_time = ros::Time::now();

    double dt = (curr_time - last_time_).toSec();
    double delta_x = (vx_ * cos(th_) - vy_ * sin(th_)) * dt;
    double delta_y = (vx_ * sin(th_) + vy_ * cos(th_)) * dt;
    double delta_th = vth_ * dt;

    x_ += delta_x;
    y_ += delta_y;
    th_ += delta_th;
    last_time_ = curr_time;               

	return true;
}


bool agv::spinOnce()
{
    // 读取机器人实际速度
    readSpeed();

    current_time_ = ros::Time::now();
    // 发布TF
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time_;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id  = "base_footprint";

    geometry_msgs::Quaternion odom_quat;
    odom_quat = tf::createQuaternionMsgFromYaw(th_);
    odom_trans.transform.translation.x = x_;
    odom_trans.transform.translation.y = y_;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;
    
    odom_broadcaster_.sendTransform(odom_trans);

    // 发布里程计消息
    nav_msgs::Odometry msgl;
    msgl.header.stamp = current_time_;
    msgl.header.frame_id = "odom";

    msgl.pose.pose.position.x = x_;
    msgl.pose.pose.position.y = y_;
    msgl.pose.pose.position.z = 0.0;
    msgl.pose.pose.orientation = odom_quat;
    msgl.pose.covariance = odom_pose_covariance;

    msgl.child_frame_id = "base_footprint";
    msgl.twist.twist.linear.x = vx_;
    msgl.twist.twist.linear.y = vy_;
    msgl.twist.twist.angular.z = vth_;
    msgl.twist.covariance = odom_twist_covariance;
  
    pub_.publish(msgl);
}

void agv::laser_callback(const sensor_msgs::LaserScanConstPtr &msg)
{
}

void agv::motorstate_callback(const omh_robot_base::MotorState & msg)
{
    if(&msg != nullptr)
    {
        omega_left_ = -msg.motor_l.actual_vel * 2 * MY_PI / GEAR_RATIO;// agv 左轮角速度 rad/s
        omega_right_ = msg.motor_r.actual_vel * 2 * MY_PI / GEAR_RATIO;// agv 右轮角速度 rad/s

        vel_left_ = omega_left_ * left_wheel_radius_;// 左轮线速度 m/s
        vel_right_ = omega_right_ * right_wheel_radius_;// 右轮线速度 m/s
    }
}

}// end of namespace agv
