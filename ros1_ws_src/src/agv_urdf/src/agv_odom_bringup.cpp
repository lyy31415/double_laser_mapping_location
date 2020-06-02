#include "agv_urdf/agv_odom.h"
  
int main(int argc, char** argv)
{
    //初始化ROS节点
	ros::init(argc, argv, "agv_bringup");									
    ros::NodeHandle nh;
    
    //初始化agv
	agv::agv robot;
    if(!robot.init())
        ROS_ERROR("agv initialized failed.");
	ROS_INFO("agv initialized successful.");
    

    //循环运行
    ros::Rate loop_rate(1000);
	while (ros::ok()) 
    {
		ros::spinOnce();
        
        // 机器人控制
        robot.spinOnce();
        
		loop_rate.sleep();
	}

	return 0;
}

