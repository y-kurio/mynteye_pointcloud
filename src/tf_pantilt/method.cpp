#include<mynteye_pointcloud/tf_pantilt.h>

void BroadCaster::__pan_anglecallback(const geometry_msgs::PoseStampedConstPtr& msg)
{
   pan_angle_ = *msg;
    // __tf_pantilt();
}

void BroadCaster::mainloop()
{
    // ros::spin();

    ros::Rate loop_rate(30);
    while (ros::ok())
    {
        __tf_pantilt();
        ros::spinOnce();
        loop_rate.sleep();
    }
}

void BroadCaster::__tf_pantilt()
{
    ROS_INFO("abcdefg");
    double roll, pitch, yaw;
    potbot_lib::utility::get_RPY(pan_angle_.pose.orientation, roll, pitch, yaw);
    ROS_INFO("roll = %f, pitch = %f, yaw = %f", roll, pitch, yaw);
    
    geometry_msgs::TransformStamped transformStamped_pan;
    std_msgs::Header time;
    time.stamp = ros::Time::now();
    transformStamped_pan.header.stamp = time.stamp;
    transformStamped_pan.header.frame_id = "robot_4/camera_stay_link";
    transformStamped_pan.child_frame_id = "robot_4/pan_link";
    transformStamped_pan.transform.translation.x = 0.05;
    transformStamped_pan.transform.translation.y = 0.0;
    transformStamped_pan.transform.translation.z = 0.2;
    transformStamped_pan.transform.rotation = potbot_lib::utility::get_Quat(0, 0, yaw);
    dynamic_br_pan_.sendTransform(transformStamped_pan);

    geometry_msgs::TransformStamped transformStamped_tilt;
    transformStamped_tilt.header.stamp = time.stamp;
    transformStamped_tilt.header.frame_id = "robot_4/pan_link";
    transformStamped_tilt.child_frame_id = "robot_4/tilt_link";
    transformStamped_tilt.transform.translation.x = 0.0;
    transformStamped_tilt.transform.translation.y = -0.08;
    transformStamped_tilt.transform.translation.z = 0.025;
    transformStamped_tilt.transform.rotation = potbot_lib::utility::get_Quat(0, pitch, 0);
    dynamic_br_tilt_.sendTransform(transformStamped_tilt);
}