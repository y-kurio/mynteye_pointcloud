#include<mynteye_pointcloud/tf_pantilt.h>

void BroadCaster::__pan_anglecallback(const geometry_msgs::PoseStampedConstPtr& msg)
{
   pan_angle_ = *msg;
    __tf_pantilt();
}

void BroadCaster::__tf_pantilt()
{
    ROS_INFO("abcdefg");
    double roll, pitch, yaw;
    potbot_lib::utility::get_RPY(pan_angle_.pose.orientation, roll, pitch, yaw);
    ROS_INFO("ab = %f, cd = %f, efg = %f", roll, pitch, yaw);
    
    geometry_msgs::TransformStamped transformStamped_pan;
    transformStamped_pan.header.stamp = pan_angle_.header.stamp;
    transformStamped_pan.header.frame_id = "robot4/camera_stay_link";
    transformStamped_pan.child_frame_id = "robot4/pan_link";
    transformStamped_pan.transform.translation.x = 0.0;
    transformStamped_pan.transform.translation.y = 0.0;
    transformStamped_pan.transform.translation.z = 0.0;
    transformStamped_pan.transform.rotation = potbot_lib::utility::get_Quat(0, 0, yaw);
    dynamic_br_pan_.sendTransform(transformStamped_pan);

    geometry_msgs::TransformStamped transformStamped_tilt;
    transformStamped_tilt.header.stamp = pan_angle_.header.stamp;
    transformStamped_tilt.header.frame_id = "robot4/pan_link";
    transformStamped_tilt.child_frame_id = "robot4/tilt_link";
    transformStamped_tilt.transform.translation.x = 0.0;
    transformStamped_tilt.transform.translation.y = 0.0;
    transformStamped_tilt.transform.translation.z = 0.0;
    transformStamped_tilt.transform.rotation = potbot_lib::utility::get_Quat(0, pitch, 0);
    dynamic_br_tilt_.sendTransform(transformStamped_tilt);
}