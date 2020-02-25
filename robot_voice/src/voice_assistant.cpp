#include <iostream>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <time.h>   

#include "robot_voice/qtts.h"
#include "robot_voice/msp_cmn.h"
#include "robot_voice/msp_errors.h"

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"

#include <sstream>
#include <sys/types.h>
#include <sys/stat.h>

#include <stdio.h>
#include <serial/serial.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <boost/asio.hpp>
#include <move_base_msgs/MoveBaseActionGoal.h>
#include "std_msgs/String.h"

static geometry_msgs::Twist run_cmdvel;
static ros::Publisher cmdvel_pub;
static ros::Publisher wakeupstop_assist;
static ros::Publisher movebase_goal_pub;
static ros::Publisher goalResultPub;
static bool woshi_goal_flag = false;
static bool keting_goal_flag = false;
static bool shafa_goal_flag = false;
static bool tv_goal_flag = false;


/* wav音频头部格式 */
typedef struct _wave_pcm_hdr
{
	char            riff[4];                // = "RIFF"
	int		size_8;                 // = FileSize - 8
	char            wave[4];                // = "WAVE"
	char            fmt[4];                 // = "fmt "
	int		fmt_size;		// = 下一个结构体的大小 : 16

	short int       format_tag;             // = PCM : 1
	short int       channels;               // = 通道数 : 1
	int		samples_per_sec;        // = 采样率 : 8000 | 6000 | 11025 | 16000
	int		avg_bytes_per_sec;      // = 每秒字节数 : samples_per_sec * bits_per_sample / 8
	short int       block_align;            // = 每采样点字节数 : wBitsPerSample / 8
	short int       bits_per_sample;        // = 量化比特数: 8 | 16

	char            data[4];                // = "data";
	int		data_size;              // = 纯数据长度 : FileSize - 44 
} wave_pcm_hdr;

/* 默认wav音频头部数据 */
wave_pcm_hdr default_wav_hdr = 
{
	{ 'R', 'I', 'F', 'F' },
	0,
	{'W', 'A', 'V', 'E'},
	{'f', 'm', 't', ' '},
	16,
	1,
	1,
	16000,
	32000,
	2,
	16,
	{'d', 'a', 't', 'a'},
	0  
};
/* 文本合成 */
int text_to_speech(const char* src_text, const char* des_path, const char* params)
{
	int          ret          = -1;
	FILE*        fp           = NULL;
	const char*  sessionID    = NULL;
	unsigned int audio_len    = 0;
	wave_pcm_hdr wav_hdr      = default_wav_hdr;
	int          synth_status = MSP_TTS_FLAG_STILL_HAVE_DATA;

	if (NULL == src_text || NULL == des_path)
	{
		printf("params is error!\n");
		return ret;
	}
	fp = fopen(des_path, "wb");
	if (NULL == fp)
	{
		printf("open %s error.\n", des_path);
		return ret;
	}
	/* 开始合成 */
	sessionID = QTTSSessionBegin(params, &ret);
	if (MSP_SUCCESS != ret)
	{
		printf("QTTSSessionBegin failed, error code: %d.\n", ret);
		fclose(fp);
		return ret;
	}
	ret = QTTSTextPut(sessionID, src_text, (unsigned int)strlen(src_text), NULL);
	if (MSP_SUCCESS != ret)
	{
		printf("QTTSTextPut failed, error code: %d.\n",ret);
		QTTSSessionEnd(sessionID, "TextPutError");
		fclose(fp);
		return ret;
	}
	printf("正在合成 ...\n");
	fwrite(&wav_hdr, sizeof(wav_hdr) ,1, fp); //添加wav音频头，使用采样率为16000
	while (1) 
	{
		/* 获取合成音频 */
		const void* data = QTTSAudioGet(sessionID, &audio_len, &synth_status, &ret);
		if (MSP_SUCCESS != ret)
			break;
		if (NULL != data)
		{
			fwrite(data, audio_len, 1, fp);
		    wav_hdr.data_size += audio_len; //计算data_size大小
		}
		if (MSP_TTS_FLAG_DATA_END == synth_status)
			break;
		printf(">");
		usleep(150*1000); //防止频繁占用CPU
	}//合成状态synth_status取值请参阅《讯飞语音云API文档》
	printf("\n");
	if (MSP_SUCCESS != ret)
	{
		printf("QTTSAudioGet failed, error code: %d.\n",ret);
		QTTSSessionEnd(sessionID, "AudioGetError");
		fclose(fp);
		return ret;
	}
	/* 修正wav文件头数据的大小 */
	wav_hdr.size_8 += wav_hdr.data_size + (sizeof(wav_hdr) - 8);
	
	/* 将修正过的数据写回文件头部,音频文件为wav格式 */
	fseek(fp, 4, 0);
	fwrite(&wav_hdr.size_8,sizeof(wav_hdr.size_8), 1, fp); //写入size_8的值
	fseek(fp, 40, 0); //将文件指针偏移到存储data_size值的位置
	fwrite(&wav_hdr.data_size,sizeof(wav_hdr.data_size), 1, fp); //写入data_size的值
	fclose(fp);
	fp = NULL;
	/* 合成完毕 */
	ret = QTTSSessionEnd(sessionID, "Normal");
	if (MSP_SUCCESS != ret)
	{
		printf("QTTSSessionEnd failed, error code: %d.\n",ret);
	}

	return ret;
}

std::string to_string(int val) 
{
    char buf[20];
    sprintf(buf, "%d", val);
    return std::string(buf);
}


void voiceWordsCallback(const std_msgs::String::ConstPtr& msg)
{
    char cmd[2000];
    const char* text;
    int         ret                  = MSP_SUCCESS;
    const char* session_begin_params = "voice_name = xiaowanzi, text_encoding = utf8, sample_rate = 16000, speed = 70, volume = 70, pitch = 10, rdn = 0";
    const char* filename             = "tts_sample.wav"; //合成的语音文件名称
    std_msgs::Int32 msg_t;

    
    run_cmdvel.linear.x = 0.0;
    cmdvel_pub.publish(run_cmdvel);

    std::cout<<"I heard :"<<msg->data.c_str()<<std::endl;

    std::string dataString = msg->data;

    msg_t.data = 1;// 停止继续唤醒
    wakeupstop_assist.publish(msg_t);

    if(dataString.compare("你是谁？") == 0)
    {
	char nameString[60] = "我在！";
        text = nameString;
        std::cout<<text<<std::endl;
    }
    else if(dataString.compare("你可以做什么？") == 0)
    {
	char helpString[60] = "我什么都能干，请您吩咐吧！";
        text = helpString;
        std::cout<<text<<std::endl;
    }
    else if(dataString.compare("现在时间。") == 0)
    {
        //获取当前时间
        struct tm *ptm; 
        long ts; 
	char timeString[60];
        ts = time(NULL); 
        ptm = localtime(&ts); 
        std::string string_t = "现在时间是" + to_string(ptm-> tm_hour) + "点" + to_string(ptm-> tm_min) + "分";

        memcpy(timeString, string_t.c_str(), sizeof(string_t));
        text = timeString;
        std::cout<<text<<std::endl;
    }
    else if(dataString.compare("出来吧。") == 0)
    {
	char outString[60] = "铁蛋出来了，您有什么吩咐？";
        text = outString;
        std::cout<<text<<std::endl;
	for(char i=0;i<5;i++)
	{
	    run_cmdvel.linear.x = 0.2;
	    cmdvel_pub.publish(run_cmdvel);
	}
	
    }
    else if(dataString.compare("回去吧。") == 0)
    {
	char inString[60] = "那铁蛋先回去了，随时等候安排！";
        text = inString;
        std::cout<<text<<std::endl;

        for(char i=0;i<5;i++)
	{
	    run_cmdvel.linear.x = -0.2;
	    cmdvel_pub.publish(run_cmdvel);
	}

    }
    else if(dataString.compare("唱首歌吧。") == 0)
    {
	char singString[60] = "天青色等烟雨，而我在等你！";
        text = singString;
        std::cout<<text<<std::endl;

    }
    else if(dataString.compare("铁蛋找不到地方，快帮帮我！") == 0)
    {
	char failString[60] = "铁蛋找不到地方，快帮帮我！";
        text = failString;
        std::cout<<text<<std::endl;

    }
    else if(dataString.compare("铁蛋成功到达，请您吩咐！") == 0)
    {
	char succeedString[60] = "铁蛋成功到达，请您吩咐！";
        text = succeedString;
        std::cout<<text<<std::endl;

    }
    else if(dataString.compare("去卧室。") == 0)
    {
	char ketingString[60] = "好的，铁蛋准备去卧室！";
        text = ketingString;
        std::cout<<text<<std::endl;

	woshi_goal_flag = true;
    }
    else if(dataString.compare("去沙发。") == 0)
    {
	char cantingString[60] = "好的，铁蛋准备去沙发！";
        text = cantingString;
        std::cout<<text<<std::endl;

	shafa_goal_flag = true;

    }
    else if(dataString.compare("去客厅。") == 0)
    {
	char cantingString[60] = "好的，铁蛋准备去客厅！";
        text = cantingString;
        std::cout<<text<<std::endl;

	keting_goal_flag = true;

    }
    else if(dataString.compare("去看电视吧。") == 0)
    {
	char cantingString[60] = "好的，铁蛋准备去看电视！";
        text = cantingString;
        std::cout<<text<<std::endl;

	tv_goal_flag = true;

    }
    else
    {
        text = msg->data.c_str();
    }

    /* 文本合成 */
    printf("开始合成 ...\n");
    ret = text_to_speech(text, filename, session_begin_params);
    if (MSP_SUCCESS != ret)
    {
        printf("text_to_speech failed, error code: %d.\n", ret);
    }
    printf("合成完毕\n");


    unlink("/tmp/cmd");  
    mkfifo("/tmp/cmd", 0777);  
    popen("mplayer -quiet -slave -input file=/tmp/cmd 'tts_sample.wav'","r");
    sleep(4);

    printf("voiceWordsCallback: msg_t.data = 0 开始继续唤醒\n");
    msg_t.data = 0;// 开始继续唤醒
    wakeupstop_assist.publish(msg_t);
}

void toExit()
{
    printf("按任意键退出 ...\n");
    getchar();
    MSPLogout(); //退出登录
}

int main(int argc, char* argv[])
{
	int         ret                  = MSP_SUCCESS;
	//const char* login_params         = "appid = 594a7b46, work_dir = .";//登录参数,appid与msc库绑定,请勿随意改动
	const char* login_params         = "appid = 5e3d6c83, work_dir = .";//登录参数,appid与msc库绑定,请勿随意改动
	/*
	* rdn:           合成音频数字发音方式
	* volume:        合成音频的音量
	* pitch:         合成音频的音调
	* speed:         合成音频对应的语速
	* voice_name:    合成发音人
	* sample_rate:   合成音频采样率
	* text_encoding: 合成文本编码格式
	*
	* 详细参数说明请参阅《讯飞语音云MSC--API文档》
	*/

	/* 用户登录 */
	ret = MSPLogin(NULL, NULL, login_params);//第一个参数是用户名，第二个参数是密码，第三个参数是登录参数，用户名和密码可在http://open.voicecloud.cn注册获取
	if (MSP_SUCCESS != ret)
	{
		printf("MSPLogin failed, error code: %d.\n", ret);
		/*goto exit ;*///登录失败，退出登录
        	toExit();
	}

	printf("\n###########################################################################\n");
	printf("## 语音合成（Text To Speech，TTS）技术能够自动将任意文字实时转换为连续的 ##\n");
	printf("## 自然语音，是一种能够在任何时间、任何地点，向任何人提供语音信息服务的  ##\n");
	printf("## 高效便捷手段，非常符合信息时代海量数据、动态更新和个性化查询的需求。  ##\n");
	printf("###########################################################################\n\n");


    ros::init(argc,argv,"TextToSpeech");
    ros::NodeHandle n;
    ros::Subscriber sub =n.subscribe("voiceWords", 10,voiceWordsCallback);
    
    wakeupstop_assist = n.advertise<std_msgs::Int32>("/voice_system/stopwakeup", 1000);
    movebase_goal_pub = n.advertise<move_base_msgs::MoveBaseActionGoal>("/move_base/goal", 1000);


    //发布主题
    cmdvel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel_serial",100);

    ros::Rate loop_rate(50);
    while(ros::ok())
    {



	if(keting_goal_flag == true)
	{
	    keting_goal_flag = false;
    	    move_base_msgs::MoveBaseActionGoal keting_goal;
	    keting_goal.goal.target_pose.header.frame_id = "map";
	    keting_goal.goal.target_pose.pose.position.x = 1.56649255753;
	    keting_goal.goal.target_pose.pose.position.y = 0.773027181625;
	    keting_goal.goal.target_pose.pose.position.z = 0.0;

	    keting_goal.goal.target_pose.pose.orientation.x = 0.0;
	    keting_goal.goal.target_pose.pose.orientation.y = 0.0;
	    keting_goal.goal.target_pose.pose.orientation.z = 0.959695447933;
	    keting_goal.goal.target_pose.pose.orientation.w = -0.281042073747;

	    movebase_goal_pub.publish(keting_goal);
	    printf("发布铁蛋去客厅\n");
	}


	if(woshi_goal_flag == true)
	{
	    woshi_goal_flag = false;
	    move_base_msgs::MoveBaseActionGoal canting_goal;
	    canting_goal.goal.target_pose.header.frame_id = "map";
	    canting_goal.goal.target_pose.pose.position.x = 0.348333120346;
	    canting_goal.goal.target_pose.pose.position.y = -1.33647680283;
	    canting_goal.goal.target_pose.pose.position.z = 0.0;

	    canting_goal.goal.target_pose.pose.orientation.x = 0.0;
	    canting_goal.goal.target_pose.pose.orientation.y = 0.0;
	    canting_goal.goal.target_pose.pose.orientation.z = -0.830408235828;
	    canting_goal.goal.target_pose.pose.orientation.w = 0.557155419851;

	    movebase_goal_pub.publish(canting_goal);
	    printf("发布铁蛋去卧室\n");
	}


	if(shafa_goal_flag == true)
	{
	    shafa_goal_flag = false;
	    move_base_msgs::MoveBaseActionGoal canting_goal;
	    canting_goal.goal.target_pose.header.frame_id = "map";
	    canting_goal.goal.target_pose.pose.position.x = 2.74950647354;
	    canting_goal.goal.target_pose.pose.position.y = 1.41340196133;
	    canting_goal.goal.target_pose.pose.position.z = 0.0;

	    canting_goal.goal.target_pose.pose.orientation.x = 0.0;
	    canting_goal.goal.target_pose.pose.orientation.y = 0.0;
	    canting_goal.goal.target_pose.pose.orientation.z = 0.866209723681;
	    canting_goal.goal.target_pose.pose.orientation.w = 0.499680612592;

	    movebase_goal_pub.publish(canting_goal);
	    printf("发布铁蛋去沙发\n");
	}


	if(tv_goal_flag == true)
	{
	    tv_goal_flag = false;
	    move_base_msgs::MoveBaseActionGoal canting_goal;
	    canting_goal.goal.target_pose.header.frame_id = "map";
	    canting_goal.goal.target_pose.pose.position.x = 0.941064476967;
	    canting_goal.goal.target_pose.pose.position.y = 1.88892126083;
	    canting_goal.goal.target_pose.pose.position.z = 0.0;

	    canting_goal.goal.target_pose.pose.orientation.x = 0.0;
	    canting_goal.goal.target_pose.pose.orientation.y = 0.0;
	    canting_goal.goal.target_pose.pose.orientation.z = 0.888501255319;
	    canting_goal.goal.target_pose.pose.orientation.w = 0.458874186784;

	    movebase_goal_pub.publish(canting_goal);
	    printf("发布铁蛋去TV\n");
	}


	ros::spinOnce();
        loop_rate.sleep();

    }

exit:
	//printf("按任意键退出 ...\n");
	//getchar();
	MSPLogout(); //退出登录

	return 0;
}

