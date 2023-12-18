#include <ros/ros.h>
#include <cstdio>
#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/Float64.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <string.h>
#include <geometry_msgs/PoseStamped.h>
#include <potbot_lib/Utility.h>


class BroadCaster
{
    private:
        ros::NodeHandle nh_;
        ros::Timer timer_;
        tf2_ros::TransformBroadcaster dynamic_br_pan_, dynamic_br_tilt_;
        ros::Subscriber sub_Cluster_point_, sub_pan_angle_;
        std::string CAMERA_STAY_LINK, PAN_LINK, TILTLINK;
        geometry_msgs::PoseStamped pan_angle_;
        void __pan_anglecallback(const geometry_msgs::PoseStampedConstPtr& msg);
        void __tf_pantilt();
        void __setLaunchParam();
    public:
        BroadCaster();
        ~BroadCaster();
};