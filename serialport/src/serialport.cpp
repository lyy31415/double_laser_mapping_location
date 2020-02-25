#include "/home/lyy/catkin_ws/src/serialport/include/serialPort/communicate_stm32.h"

static float RobotV_ = 0;
static float YawRate_ = 0;
static bool pos_clear = false;

// 速度控制消息的回调函数
void cmdCallback(const geometry_msgs::Twist& msg)
{
	RobotV_ = msg.linear.x;
	YawRate_ = msg.angular.z;
}

void clearCallback(const std_msgs::Bool& msg)
{
	if(msg.data == true)  pos_clear = true;
	else                  pos_clear = false;
}


int main(int argc, char** argv)
{
	//初始化节点
	ros::init(argc, argv, "serial_node_lyy");

	//声明节点句柄
	ros::NodeHandle nh;

	//订阅主题，并配置回调函数
    ros::Subscriber sub = nh.subscribe("cmd_vel_serial", 50, cmdCallback);
    ros::Subscriber sub_clear = nh.subscribe("position_clear", 50, clearCallback);


    //初始化 Dreamcar
	dreamcar::dreamcar robot;
    if(!robot.init())
        ROS_ERROR("Dreamcar initialized failed.");
	ROS_INFO("Dreamcar initialized successful.");
    


    //循环运行
    ros::Rate loop_rate(50);
	while (ros::ok()) 
	{
	ros::spinOnce();
        
        // 机器人控制
        robot.spinOnce(RobotV_, YawRate_);
	robot.clear_flag(pos_clear);
	if(robot.fun_pos_clear() == true)  pos_clear = false;        
	loop_rate.sleep();
	}

	return 0;
}






