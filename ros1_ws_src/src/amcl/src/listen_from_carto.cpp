#include "ros/ros.h"
#include "amcl/pose_test.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2/convert.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/message_filter.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "message_filters/subscriber.h"
#include <iostream>
#include <boost/bind.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread.hpp>

#define RAD_TO_DEGREE          57.29578

class listenfromcarto
{
    public:
        listenfromcarto();

        std::shared_ptr<tf2_ros::TransformListener> tfl_;
        std::shared_ptr<tf2_ros::Buffer> tf_;
        ros::Publisher pose_test_pub_;
        std::string base_frame_id_;
        std::string global_frame_id_;
        ros::NodeHandle nh_;
        ros::Time old_print_time_;
};

listenfromcarto::listenfromcarto():base_frame_id_("base_link"), global_frame_id_("map"), nh_("~")
{
    pose_test_pub_ = nh_.advertise<amcl::pose_test>("carto_pose", 1000, true);
    tf_.reset(new tf2_ros::Buffer());
    tfl_.reset(new tf2_ros::TransformListener(*tf_));
}

boost::shared_ptr<listenfromcarto> listenfromcarto_node_ptr;


int main(int argc, char** argv)
{
    ros::init(argc, argv, "listen_from_carto");
    
    listenfromcarto_node_ptr.reset(new listenfromcarto());
  
    geometry_msgs::PoseWithCovarianceStamped map_pose;
    geometry_msgs::PoseWithCovarianceStamped ident;
    amcl::pose_test carto_pose;

    ros::Rate loop_rate(500);
    while (ros::ok())
    {
        ros::Time now = ros::Time::now();
        ident.header.frame_id = listenfromcarto_node_ptr->base_frame_id_;
        ident.header.stamp = now;
        tf2::toMsg(tf2::Transform::getIdentity(), ident.pose.pose);

        try
        {
            listenfromcarto_node_ptr->tf_->transform(ident, map_pose, listenfromcarto_node_ptr->global_frame_id_,
                                                    ros::Duration(0.5));
        }
        catch(tf2::TransformException e)
        {
            ROS_ERROR("Cannot to get transform between map and baselink at %f!", now.toSec());
        }

        carto_pose.header = map_pose.header;
        carto_pose.x = map_pose.pose.pose.position.x;
        carto_pose.y = map_pose.pose.pose.position.y;
        carto_pose.theta = tf2::getYaw(map_pose.pose.pose.orientation) * RAD_TO_DEGREE;
        listenfromcarto_node_ptr->pose_test_pub_.publish(carto_pose);

        if(now.toSec() - listenfromcarto_node_ptr->old_print_time_.toSec() > 1.0)
        {
            ROS_INFO("carto_pose: %.3f m, %.3f m, %.3f deg, with stamp=%f ",
                carto_pose.x, carto_pose.y, carto_pose.theta, carto_pose.header.stamp.toSec());
            
            listenfromcarto_node_ptr->old_print_time_ = now;
        }

        loop_rate.sleep();
    }

    listenfromcarto_node_ptr.reset();
    
    return 0;
}