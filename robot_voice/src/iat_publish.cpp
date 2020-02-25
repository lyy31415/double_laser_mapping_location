/*
* 语音听写(iFly Auto Transform)技术能够实时地将语音转换成对应的文字。
*/

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include "robot_voice/qisr.h"
#include "robot_voice/msp_cmn.h"
#include "robot_voice/msp_errors.h"
#include "robot_voice/speech_recognizer.h"
#include <iconv.h>

#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "std_msgs/String.h"
#include <move_base_msgs/MoveBaseActionResult.h>

#define FRAME_LEN   640 
#define BUFFER_SIZE 4096

int wakeupFlag   = 0 ;
int resultFlag   = 0 ;
int noresultFlag = 0;// by lyy

static ros::Publisher wakeupstop_iat;
static std_msgs::Int32 msg_stop;
static bool goal_fail_flag = false;
static bool goal_succeed_flag = false;


static void show_result(char *string, char is_over)
{
    resultFlag=1;
    noresultFlag = 1;// by lyy
    printf("\rResult: [ %s ]", string);
    if(is_over)
        putchar('\n');
}

static char *g_result = NULL;
static unsigned int g_buffersize = BUFFER_SIZE;

void on_result(const char *result, char is_last)
{
    if (result) {
        size_t left = g_buffersize - 1 - strlen(g_result);
        size_t size = strlen(result);
        if (left < size) {
            g_result = (char*)realloc(g_result, g_buffersize + BUFFER_SIZE);
            if (g_result)
                g_buffersize += BUFFER_SIZE;
            else {
                printf("mem alloc failed\n");
                return;
            }
        }
        strncat(g_result, result, size);
        show_result(g_result, is_last);
    }
}

void on_speech_begin()
{
    if (g_result)
    {
        free(g_result);
    }
    g_result = (char*)malloc(BUFFER_SIZE);
    g_buffersize = BUFFER_SIZE;
    memset(g_result, 0, g_buffersize);

    printf("Start Listening...\n");
}
void on_speech_end(int reason)
{
    if (reason == END_REASON_VAD_DETECT)
        printf("\nSpeaking done \n");
    else
        printf("\nRecognizer error %d\n", reason);
}

/* demo recognize the audio from microphone */
static void demo_mic(const char* session_begin_params)
{
    int errcode;
    int i = 0;

    struct speech_rec iat;

    struct speech_rec_notifier recnotifier = {
        on_result,
        on_speech_begin,
        on_speech_end
    };

    noresultFlag = 0;// by lyy
    errcode = sr_init(&iat, session_begin_params, SR_MIC, &recnotifier);
    if (errcode) {
        printf("speech recognizer init failed\n");
        return;
    }
    errcode = sr_start_listening(&iat);
    if (errcode) {
        printf("start listen failed %d\n", errcode);
    }
    /* demo 10 seconds recording */
    while(i++ < 2)
        sleep(1);
    errcode = sr_stop_listening(&iat);
    if (errcode) {
        printf("stop listening failed %d\n", errcode);
    }

    sr_uninit(&iat);
}


/* main thread: start/stop record ; query the result of recgonization.
 * record thread: record callback(data write)
 * helper thread: ui(keystroke detection)
 */

void WakeUp(const std_msgs::Int32::ConstPtr& msg)
{
    if(msg->data == 1)
    {
	    //usleep(700*1000);
	    usleep(900*1000);
	    printf("waking up\r\n");
	    wakeupFlag=1;

	    msg_stop.data = 1;// 停止继续唤醒
	    wakeupstop_iat.publish(msg_stop);
	    printf("WakeUp: msg_stopg.data = 1 停止继续唤醒\n");
    }
    else
    {
        wakeupFlag=0;
    }  


}



void goalSubCallback(const move_base_msgs::MoveBaseActionResult::ConstPtr& msg)
{
    //if(msg->status.status == 4)  goal_fail_flag = true;
    //else                         goal_fail_flag = false;

    if(msg->status.status == 3)  {goal_succeed_flag = true;   goal_fail_flag = false;}
    else                         {goal_succeed_flag = false;  goal_fail_flag = true;}

    printf("goal result status: %d\n",msg->status.status);
}



int main(int argc, char* argv[])
{
    // 初始化ROS
    ros::init(argc, argv, "voiceRecognition");
    ros::NodeHandle n;
    ros::Rate loop_rate(30);

    // 声明Publisher和Subscriber
    // 订阅唤醒语音识别的信号
    ros::Subscriber wakeUpSub = n.subscribe("/voice_system/voiceWakeup", 2, WakeUp);
    ros::Subscriber goal_sub =n.subscribe("/move_base/result", 10,goalSubCallback);   
    // 订阅唤醒语音识别的信号    
    ros::Publisher voiceWordsPub = n.advertise<std_msgs::String>("voiceWords", 10);
    wakeupstop_iat = n.advertise<std_msgs::Int32>("/voice_system/stopwakeup", 10);

    std_msgs::String mytext;

    ROS_INFO("Sleeping...");
    //int count=0;
    while(ros::ok())
    {
        // 语音识别唤醒
        if (wakeupFlag){
            ROS_INFO("Wakeup...");
            int ret = MSP_SUCCESS;
            //const char* login_params = "appid = 594a7b46, work_dir = .";
	    const char* login_params = "appid = 5e3d6c83, work_dir = .";

            const char* session_begin_params =
                "sub = iat, domain = iat, language = zh_cn, "
                "accent = mandarin, sample_rate = 16000, "
                "result_type = plain, result_encoding = utf8";

            ret = MSPLogin(NULL, NULL, login_params);
            if(MSP_SUCCESS != ret){
                MSPLogout();
                printf("MSPLogin failed , Error code %d.\n",ret);
            }

            printf("Demo recognizing the speech from microphone\n");
            printf("Speak in 3 seconds\n");

            demo_mic(session_begin_params);
	    if(noresultFlag == 0)
	    {
		msg_stop.data = 0;// 继续唤醒
		wakeupstop_iat.publish(msg_stop);
		printf("没有检测到语音，开始继续唤醒\n");
	    }
            printf("3 sec passed\n");
        
            wakeupFlag=0;
            MSPLogout();
        }

        // 语音识别完成
        if(resultFlag){
            resultFlag=0;
            std_msgs::String msg;
            msg.data = g_result;
            voiceWordsPub.publish(msg);

        }


	if(goal_fail_flag == true)
	{	    
	    goal_fail_flag = false;

	    char failString[80] = "铁蛋找不到地方，快帮帮我！";
            mytext.data = failString;
            voiceWordsPub.publish(mytext);
	    printf("铁蛋找不到地方，快帮帮我！\n");
	}

	if(goal_succeed_flag == true)
	{
	    goal_succeed_flag = false;

            char succeedString[80] = "铁蛋成功到达，请您吩咐！";
            mytext.data = succeedString;
            voiceWordsPub.publish(mytext);
	    printf("铁蛋成功到达，请您吩咐！\n");
	}

        ros::spinOnce();
        loop_rate.sleep();
        //count++;
    }

exit:
    MSPLogout(); // Logout...

    return 0;
}
