#include <vector>
#include "/home/lyy/catkin_ws/src/serialport/include/serialPort/communicate_stm32.h"


namespace dreamcar
{

boost::array<float, 36> odom_pose_covariance = {
    {1e-9, 0, 0, 0, 0, 0, 
    0, 1e-3, 1e-9, 0, 0, 0, 
    0, 0, 1e6, 0, 0, 0,
    0, 0, 0, 1e6, 0, 0, 
    0, 0, 0, 0, 1e6, 0, 
    0, 0, 0, 0, 0, 1e-9}};
boost::array<float, 36> odom_twist_covariance = {
    {1e-9, 0, 0, 0, 0, 0, 
    0, 1e-3, 1e-9, 0, 0, 0, 
    0, 0, 1e6, 0, 0, 0, 
    0, 0, 0, 1e6, 0, 0, 
    0, 0, 0, 0, 1e6, 0, 
    0, 0, 0, 0, 0, 1e-9}};



dreamcar::dreamcar():
    x_(0.0), y_(0.0), th_(0.0),
    vx_(0.0), vy_(0.0), vth_(0.0)
{
    send_len = PC2MCULEN;
	send_initdata = 0;
	send_temp = &send_initdata;
	pos_clear_flag = false;
}

dreamcar::~dreamcar()
{
}

bool dreamcar::init()
{   
    ros::Time::init();
    current_time_ = ros::Time::now();
	
    //定义发布消息的名称
    pub_ = nh.advertise<nav_msgs::Odometry>("odom", 50);
    robot_cmd_pub = nh.advertise<geometry_msgs::Twist>("robot_cmd_vel", 50);		


	try
    {
		//设置串口属性，并打开串口
		ser.setPort("/dev/ch340_0");
		//ser.setBaudrate(460800);
		ser.setBaudrate(230400);
		serial::Timeout to = serial::Timeout::simpleTimeout(1000);
		ser.setTimeout(to);
		ser.open();
	}

	catch(serial::IOException & e)
    {
		ROS_ERROR("Unable to open serial port!");
		return false;
	}

	//检测串口是否已经打开，并给出提示信息
	if(ser.isOpen())
	{
		ROS_INFO("Serial port initialized.");
	}
	else
	{
		return false;
	}


    
    return true;
}



bool dreamcar::spinOnce(float RobotV, float YawRate)
{
	send_to_stm(RobotV,YawRate);
	ser.write(send_temp,send_len);//send data

	if(ser.available())
	{	
		ser.read(read_buf, READBUFLEN);
		recieve_from_stm();// recieve data
	
		printf(" Ultrasonic_dist1: %f       mm\n", recieve_temp[0]);
		printf(" Ultrasonic_dist2: %f       mm\n", recieve_temp[1]);
		printf(" Ultrasonic_dist3: %f       mm\n", recieve_temp[2]);
		printf(" Ultrasonic_dist4: %f       mm\n", recieve_temp[3]);

		printf(" Current velocity: %f      m/s\n", recieve_temp[4]);
		printf(" Current omega   : %f degree/s\n", recieve_temp[5]);
		printf(" Left omega:       %f    rad/s\n", recieve_temp[6]);
		printf(" Right omega:      %f    rad/s\n", recieve_temp[7]);
		printf(" Pos_x:            %f    meter\n", recieve_temp[8]);
		printf(" Pos_y:            %f    meter\n", recieve_temp[9]);
		printf(" Pos_theta:        %f   degree\n", recieve_temp[10]);
		printf(" BatteryVoltage:   %f     volt\n", recieve_temp[11]);
		printf(" Temperature:      %f        C\n", recieve_temp[12]);

		printf("\n\n");

		vx_ = recieve_temp[4];
		vth_ = recieve_temp[5];
		x_ = recieve_temp[8];
		y_ = recieve_temp[9];
		th_ = recieve_temp[10] / 57.3;
		
	}
	else
	{
		ROS_DEBUG("Serial port no data.");
	}


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
    msgl.twist.twist.angular.z = vth_;
    msgl.twist.covariance = odom_twist_covariance;
 
    pub_.publish(msgl);

    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = vx_;
    cmd_vel.angular.z = vth_;

    robot_cmd_pub.publish(cmd_vel);
}



void dreamcar::send_to_stm(float RobotV,float YawRate)
{
	
	memset((uint8_t *)(&pc2mcuFrame),0,PC2MCULEN);
	pc2mcuFrame.frameHead= 0xAA;
	pc2mcuFrame.robot_vel_float_x	= RobotV;
	pc2mcuFrame.robot_omega_float_z = YawRate * 57.3;
	pc2mcuFrame.isPosClear = pos_clear_flag;
	
	if(PC2MCULEN>2)
	{
		pc2mcuFrame.checksum = checkSum((uint8_t *)(&pc2mcuFrame),PC2MCULEN-2);
	}
	
	pc2mcuFrame.frameEnd = 0x55;
	
	send_temp=(uint8_t*)(&pc2mcuFrame);

}



void dreamcar::recieve_from_stm(void)
{
	uint8_t csum=0, pRead=0;
	
	for(pRead=0; pRead < READBUFLEN - MCU2PCLEN +1; pRead++)
	{
		if((read_buf[pRead] == 0xAA) && (read_buf[pRead+MCU2PCLEN-1] == 0x55))
		{
			if(MCU2PCLEN>2)
			{
				csum = checkSum((uint8_t *)(&read_buf[pRead]),MCU2PCLEN-2);
			}
			
			ROS_DEBUG("Data head and tail found!");
			if(read_buf[pRead+MCU2PCLEN-2] == csum)
			{
				recieve_temp[0]=*((float*)(&read_buf[pRead+2]));//  ultrasonic dist1: meter
				recieve_temp[1]=*((float*)(&read_buf[pRead+6]));//  ultrasonic dist2: meter
				recieve_temp[2]=*((float*)(&read_buf[pRead+10]));// ultrasonic dist3: meter
				recieve_temp[3]=*((float*)(&read_buf[pRead+14]));// ultrasonic dist4: meter

				recieve_temp[4]=*((float*)(&read_buf[pRead+18]));// current vel: m/s
				recieve_temp[5]=*((float*)(&read_buf[pRead+22]));// current omega: degree/s
				recieve_temp[6]=*((float*)(&read_buf[pRead+26]));//  left omega: rad/s
				recieve_temp[7]=*((float*)(&read_buf[pRead+30]));// right omega: rad/s
				recieve_temp[8]=*((float*)(&read_buf[pRead+34]));// pos_x: meter
				recieve_temp[9]=*((float*)(&read_buf[pRead+38]));// pos_y: meter
				recieve_temp[10]=*((float*)(&read_buf[pRead+42]));// pos_theta: degree
				recieve_temp[11]=*((float*)(&read_buf[pRead+46]));// batteryVoltage
				recieve_temp[12]=*((float*)(&read_buf[pRead+50]));// temperature
				
				ROS_DEBUG("Data from stm varify success!");
				break;
			}
			else
			{
				memset((uint8_t *)(recieve_temp),0,sizeof(recieve_temp));
				ROS_DEBUG("Data from stm varify failed!");
				
			}	
		}
		else
		{
			ROS_DEBUG("Data head and tail not found!");
		}
	}
	
}



uint8_t dreamcar::checkSum(uint8_t *dataBuf,uint32_t num)
{
	uint8_t sum=0;
	for(int i=1;i<num;i++)
	{
		sum+=*(dataBuf+i);
	}
	return sum;
}



void dreamcar::clear_flag(bool flag)
{
	pos_clear_flag = flag;
}


bool dreamcar::fun_pos_clear()
{
	if(x_ == 0.0 && y_ == 0.0 && th_ == 0.0)
	    return true;
	else
	    return false;
}


}// end of ns dreamcar
