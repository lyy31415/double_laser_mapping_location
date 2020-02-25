#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <snowboy_wakeup/SnowboyReconfigureConfig.h>
#include <audio_common_msgs/AudioData.h>
#include <dynamic_reconfigure/server.h>
#include <hotword_detector.h>
//#include <wiringPi.h>

#define BTN_PIN 27
volatile char btn_click = 0;
static int wakeupstop_flag = 0;

namespace snowboy_wakeup
{
    std::string beep_filename;
    std::string pre_param = "play -q --multi-threaded ";
    std::string all_param;

    //! \brief The HotwordDetectorNode class Wraps the C++ 11 Snowboy detector in a ROS node
    class HotwordDetectorNode
    {
        public:
            HotwordDetectorNode():nh_(""),nh_p_("~")
            {}

            bool initialize()
            {
                std::string resource_filename;
                if (!nh_p_.getParam("resource_filename", resource_filename))
                {
                    ROS_ERROR("Mandatory parameter 'common.res' not present on the parameter server");
                    return false;
                }

                std::string model_filename;
                if (!nh_p_.getParam("model_filename", model_filename))
                {
                    ROS_ERROR("Mandatory parameter 'model_filename' not present on the parameter server");
                    return false;
                }

                if (!nh_p_.getParam("beep_filename", beep_filename))
                {
                    ROS_WARN("Mandatory parameter 'beep_filename' not present on the parameter server");
                }
                all_param = pre_param + beep_filename;

                std::string wakeup_topic;
                if (!nh_p_.getParam("wakeup_topic", wakeup_topic))
                {
                    ROS_ERROR("Mandatory parameter 'wakeup_topic' not present on the parameter server");
                    return false;
                }

                std::string audio_topic;
                if (!nh_p_.getParam("audio_topic", audio_topic))
                {
                    ROS_ERROR("Mandatory parameter 'audio_topic' not present on the parameter server");
                    return false;
                }

                audio_sub_ = nh_.subscribe(audio_topic, 1000, &HotwordDetectorNode::audioCallback, this);
                hotword_pub_ = nh_.advertise<std_msgs::Int32>(wakeup_topic, 1);
		wakeupstop_sub_ = nh_.subscribe("/voice_system/stopwakeup", 10, &HotwordDetectorNode::wakeupstopCallback, this);

                detector_.initialize(resource_filename.c_str(), model_filename.c_str());
                dynamic_reconfigure_server_.setCallback(boost::bind(&HotwordDetectorNode::reconfigureCallback, this, _1, _2));

                return true;
            }

        private:
            ros::NodeHandle nh_;

            //! \brief nh_p_ Local nodehandle for parameters
            ros::NodeHandle nh_p_;

            ros::Subscriber audio_sub_;
            ros::Publisher hotword_pub_;
	    ros::Subscriber wakeupstop_sub_;

            //! \brief dynamic_reconfigure_server_ In order to online tune the sensitivity and audio gain
            dynamic_reconfigure::Server<SnowboyReconfigureConfig> dynamic_reconfigure_server_;

            //! \brief detector_ C++ 11 Wrapped Snowboy detect
            HotwordDetector detector_;

            //! \brief reconfigureCallback Reconfigure update for sensitiviy and audio level
            //! \param cfg The updated config
            void reconfigureCallback(SnowboyReconfigureConfig cfg, uint32_t)
            {
                detector_.configure(cfg.sensitivity, cfg.audio_gain);
                ROS_INFO("SnowboyROS ReConfigured callback init");
            }

            //! \brief audioCallback Audio stream callback
            //! \param msg The audo data
            void audioCallback(const audio_common_msgs::AudioDataConstPtr& msg)
            {
		if(wakeupstop_flag == 0)
		{        
			//wakeupstop_flag = 1;
			if (msg->data.size() != 0)
		        {
		            if ( msg->data.size() % 2 )
		            {
		                ROS_ERROR("Not an even number of bytes in this message!");
		            }

		            int16_t sample_array[msg->data.size()/2];
		            for ( size_t i = 0; i < msg->data.size(); i+=2 )
		            {
		                sample_array[i/2] = ((int16_t) (msg->data[i+1]) << 8) + (int16_t) (msg->data[i]);
		            }

		            std_msgs::Int32 hotword_msg;
			    hotword_msg.data = 0;
		            int result = detector_.runDetection( &sample_array[0], msg->data.size()/2);
		            if (result == 1 || btn_click == 1)
		            {
		                btn_click = 0;
		                ROS_INFO("wakeUp detected!");
		                hotword_msg.data = 1;
		                hotword_pub_.publish(hotword_msg);
		                system(all_param.data());
		            }
		            else if (result == -3)
		            {
		                ROS_ERROR("Hotword detector not initialized");
		            }
		            else if (result == -1)
		            {
		                ROS_ERROR("Snowboy error");
		            }
		        }
		//printf("audioCallback: wakeupstop_flag = 0 可以开始唤醒\n");
		}
		else
		{
		    //printf("audioCallback: wakeupstop_flag = 1 不能唤醒\n");
		}
		
            }


		void wakeupstopCallback(const std_msgs::Int32::ConstPtr& msg)
		{
		    if(msg->data == 0)  {wakeupstop_flag = 0;printf("wakeupstopCallback: wakeupstop_flag = 0 可以开始唤醒\n");}
		    else                {wakeupstop_flag = 1;printf("wakeupstopCallback: wakeupstop_flag = 1 不可开始唤醒\n");}

		    printf("进入 wakeupstopCallback 回调函数\n");
		}
    };
}

//click btn ISR function
void btnISR()
{
    btn_click = 1;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "snowboy_wakeup_node");
    snowboy_wakeup::HotwordDetectorNode ros_hotword_detector_node;

    //wiringPiSetup();
    //pinMode(BTN_PIN, INPUT);
    //pullUpDnControl(BTN_PIN, PUD_UP);
    //wiringPiISR(BTN_PIN, INT_EDGE_RISING, &btnISR);

    if (ros_hotword_detector_node.initialize())
    {
	ros::Rate loop_rate(10);
	while(ros::ok())
	{
		ros::spinOnce();
		loop_rate.sleep();
	}        

    }
    else
    {
        ROS_ERROR("Failed to initialize snowboy_node");
        return 1;
    }

    return 0;
}

