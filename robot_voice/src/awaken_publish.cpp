/* 语音唤醒  By lyy */

#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <errno.h>
#include <sstream>
#include <sys/types.h>
#include <sys/stat.h>


#include "robot_voice/qivw.h"
#include "robot_voice/linuxrec.h"
#include "robot_voice/formats.h"
#include "robot_voice/msp_cmn.h"
#include "robot_voice/msp_errors.h"

#include "ros/ros.h"
#include "std_msgs/String.h"


#define SAMPLE_RATE_16K     (16000)

#define DEFAULT_FORMAT	\
{\
	WAVE_FORMAT_PCM,\
	1,			\
	16000,		\
	32000,		\
	2,			\
	16,			\
	sizeof(WAVEFORMATEX)\
}

struct recorder *recorder = NULL;
static ros::Publisher wakeUpPub;

void sleep_ms(int ms)
{
	usleep(ms * 1000);
}

/* the record call back */
void record_data_cb(char *data, unsigned long len, void *user_para)
{
	int errcode = 0;
	const char *session_id = (const char *)user_para;

	if(len == 0 || data == NULL)
		return;

    errcode = QIVWAudioWrite(session_id, (const void *)data, len, MSP_AUDIO_SAMPLE_CONTINUE);
    if (MSP_SUCCESS != errcode)
    {
        printf("QIVWAudioWrite failed! error code:%d\n",errcode);
        int ret = stop_record(recorder);
        if (ret != 0) {
            printf("Stop failed! \n");
        }
        QIVWAudioWrite(session_id, NULL, 0, MSP_AUDIO_SAMPLE_LAST);
    }
}

int cb_ivw_msg_proc( const char *sessionID, int msg, int param1, int param2, const void *info, void *userData )
{
  std_msgs::String startup;
  startup.data = "start";
  if (MSP_IVW_MSG_ERROR == msg) //唤醒出错消息
  {
    printf("\n\nMSP_IVW_MSG_ERROR errCode = %d\n\n", param1);
  }else if (MSP_IVW_MSG_WAKEUP == msg) //唤醒成功消息
  {
    //printf("\n\nMSP_IVW_MSG_WAKEUP result = %s\n\n", (char*)info);
    system("play /home/lyy/catkin_ws/src/robot_voice/sources/dong.wav");
    wakeUpPub.publish(startup);
  }

  return 0;
}


void run_ivw(const char* session_begin_params)
{
	const char *session_id = NULL;
	int err_code = MSP_SUCCESS;
	char sse_hints[128];

	WAVEFORMATEX wavfmt = DEFAULT_FORMAT;
	wavfmt.nSamplesPerSec = SAMPLE_RATE_16K;
	wavfmt.nAvgBytesPerSec = wavfmt.nBlockAlign * wavfmt.nSamplesPerSec;

    //start QIVW
	session_id=QIVWSessionBegin(NULL, session_begin_params, &err_code);
	if (err_code != MSP_SUCCESS)
	{
		printf("QIVWSessionBegin failed! error code:%d\n",err_code);
		goto exit;
	}

	err_code = QIVWRegisterNotify(session_id, cb_ivw_msg_proc, NULL);
	if (err_code != MSP_SUCCESS)
	{
		snprintf(sse_hints, sizeof(sse_hints), "QIVWRegisterNotify errorCode=%d", err_code);
		printf("QIVWRegisterNotify failed! error code:%d\n",err_code);
		goto exit;
	}

    //1.create recorder
	err_code = create_recorder(&recorder, record_data_cb, (void*)session_id);
	if (recorder == NULL || err_code != 0)
    {
			printf("create recorder failed: %d\n", err_code);
			err_code = MSP_ERROR_FAIL;
			goto exit;
	}

    //2.open_recorder
	err_code = open_recorder(recorder, get_default_input_dev(), &wavfmt);
	if (err_code != 0)
    {
		printf("recorder open failed: %d\n", err_code);
		err_code = MSP_ERROR_FAIL;
		goto exit;
	}

    //3.start record
	err_code = start_record(recorder);
	if (err_code != 0) {
		printf("start record failed: %d\n", err_code);
		err_code = MSP_ERROR_FAIL;
		goto exit;
	}

	while(1)
	{
		sleep_ms(2000); //模拟人说话时间间隙，10帧的音频时长为200ms
        printf("Listening... Press Ctrl+C to exit\n");
	}
	snprintf(sse_hints, sizeof(sse_hints), "success");

exit:
	if (recorder)
        {
		if(!is_record_stopped(recorder))
			stop_record(recorder);
		close_recorder(recorder);
		destroy_recorder(recorder);
		recorder = NULL;
	}
	if (NULL != session_id)
	{
		QIVWSessionEnd(session_id, sse_hints);
	}
}


int main(int argc, char* argv[])
{
	// 初始化ROS
	ros::init(argc, argv, "voiceWakenUp");
	ros::NodeHandle n;

	int ret = MSP_SUCCESS;
	//const char *lgi_param = "appid = 5ae04b54, work_dir = .";
	const char *lgi_param = "appid = 5e3d6c83, work_dir = .";
	const char *ssb_param = "ivw_threshold=0:2000, sst=wakeup, ivw_res_path=fo|/home/lyy/catkin_ws/src/robot_voice/res/ivw/wakeupresource.jet";

	wakeUpPub = n.advertise<std_msgs::String>("voiceWakeup", 1000);

	ros::Rate loop_rate(10);
	while(ros::ok())
	{
		ret = MSPLogin(NULL, NULL, lgi_param);
		if (MSP_SUCCESS != ret)
		{
			printf("MSPLogin failed, error code: %d.\n", ret);
			MSPLogout();//登录失败，退出登录
		}
		run_ivw(ssb_param);	
		//MSPLogout(); //退出登录
		loop_rate.sleep();
	}	
exit:
  MSPLogout();
  printf("请按任意键退出...\n");
  getchar();

	return 0;
}

