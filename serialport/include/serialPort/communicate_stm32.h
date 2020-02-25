#ifndef _COMMUNICATE_STM32_H
#define _COMMUNICATE_STM32_H

#include <stdio.h>
#include <serial/serial.h>
#include <ros/ros.h>
#include <ros/time.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <boost/asio.hpp>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>


namespace dreamcar
{

#pragma pack(1)
typedef struct _PC2MCU_FRAME
{
	uint8_t frameHead;

	float  robot_vel_float_x;// m/s
	float  robot_vel_float_y;// m/s
	float  robot_omega_float_z;// degree/s

	uint8_t isPosClear:1;
	uint8_t :7;// retained
	
	uint8_t checksum;
	uint8_t frameEnd;
	
}pc2mcuFrame_Typedef;




#pragma pack(1)
typedef struct _MCU2PC_FRAME
{
	uint8_t   frameHead;

	uint8_t   isUltraStop:1;
	uint8_t   isGyroDataOk:1;
	uint8_t   isMotorErrorL:1;
	uint8_t   isMotorErrorR:1;
	uint8_t   :4;      /*retained*/

	float   ultrasonic1_dist;
	float   ultrasonic2_dist;
	float   ultrasonic3_dist;
	float   ultrasonic4_dist;

	float   curRobotVel;
	float   curRobotOmega; 
	float	leftOmega;
	float	rightOmega;
	float   pos_x;
	float   pos_y; 
	float   pos_theta;
	
	float   batteryVoltage;
	float   temperature;
	
	uint8_t checksum;
	uint8_t frameEnd;
	
}Mcu2PcFrame_Typedef;




#define MCU2PCLEN	(uint16_t)sizeof(Mcu2PcFrame_Typedef)
#define PC2MCULEN	(uint16_t)sizeof(pc2mcuFrame_Typedef)
#define READBUFLEN  (uint16_t)(2*MCU2PCLEN)



class dreamcar
{
public:
    dreamcar();
    ~dreamcar();
    bool init();
    bool spinOnce(float RobotV, float YawRate);
    void send_to_stm(float RobotV1,float YawRate1);
    void recieve_from_stm(void);
    uint8_t checkSum(uint8_t *dataBuf,uint32_t num);
    void clear_flag(bool flag);
    bool fun_pos_clear();
	
   
private:
    ros::Time current_time_, last_time_;

    float x_;
    float y_;
    float th_;

    float vx_;
    float vy_;
    float vth_;

    uint8_t send_len;
    uint8_t read_buf[READBUFLEN];
    float recieve_temp[20];
    Mcu2PcFrame_Typedef mcu2pcFrame;
    pc2mcuFrame_Typedef pc2mcuFrame;
    uint8_t  send_initdata;
    uint8_t* send_temp;
    serial::Serial ser;//声明串口对象
    bool pos_clear_flag;

    ros::NodeHandle nh;
    ros::Publisher pub_;
    ros::Publisher robot_cmd_pub;
    tf::TransformBroadcaster odom_broadcaster_;
};

}// end of namesapce dreamcar

#endif

