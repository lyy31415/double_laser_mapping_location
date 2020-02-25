#ifndef _ROS_INTERFACE_H
#define _ROS_INTERFACE_H
#include "sys.h"
#include "usart.h"



#pragma pack(1)
typedef struct _PC2MCU_FRAME
{
	uint8_t frameHead;

	float  robot_vel_float_x;
	float  robot_vel_float_y;
	float  robot_omega_float_z;
	
	uint8_t isPosClear:1;
	uint8_t   :7;      /*retained*/
	
	uint8_t checksum;
	uint8_t frameEnd;
	
}pc2mcuFrame_Typedef;




#pragma pack(1)
typedef struct _MCU2PC_FRAME
{
	uint8_t   frameHead;
	
	uint8_t 	isUltraStop:1;
	uint8_t 	isGyroDataOk:1;
	uint8_t		isMotorErrorL:1;
	uint8_t		isMotorErrorR:1;
	uint8_t   :4;      /*retained*/

	float   ultrasonic1_dist;
	float   ultrasonic2_dist;
	float   ultrasonic3_dist;
	float   ultrasonic4_dist;

	float 	curRobotVel;
	float 	curRobotOmega; 
	float		leftOmega;
	float		rightOmega;
	float   pos_x;
	float   pos_y; 
	float   pos_theta;

	float   batteryVoltage;
	float   temperature ;
	
	uint8_t checksum;
	uint8_t frameEnd;
	
}Mcu2PcFrame_Typedef;





#define PC2MCU_HEAD    0xAA
#define PC2MCU_END     0x55
#define PC2MCULEN     (int)sizeof(pc2mcuFrame_Typedef)
#define MCU2PCLEN     (int)sizeof(Mcu2PcFrame_Typedef)



void handle_data(void);
void send_to_ros(void);
void recieve_from_ros(void);
void ROS_DMA_Enable(DMA_Stream_TypeDef *DMA_Streamx,u16 ndtr);
//void ROS_Clear_IDLE_Flag(USART_TypeDef* USARTx);
uint32_t ROS_Set_Data_Num(DMA_Stream_TypeDef *DMA_Streamx, uint16_t number);
void ROS_DMA_Config(DMA_Stream_TypeDef *DMA_Stream_Tx, DMA_Stream_TypeDef *DMA_Stream_Rx);
static uint8_t checkSum(uint8_t *dataBuf,uint32_t num);


#endif

