#include<mynteye_pointcloud/Riskprediction.h>

void RiskClass::__Cluster_closest_pointcallback(const mynteye_pointcloud::pointDataConstPtr& msg)
{
    Cluster_Minpts_ = *msg;
    __pantilt_order();
}

void RiskClass::mainloop()
{
    // ros::spin();

    ros::Rate loop_rate(30);
	while (ros::ok())
	{
        __pantilt_order();
		ros::spinOnce();
		loop_rate.sleep();
	}
}

void RiskClass::__pantilt_order()
{
    double object_x = Cluster_Minpts_.Most_closest_pt.x;
    double object_y = Cluster_Minpts_.Most_closest_pt.y;

    static tf::TransformListener listener;
    geometry_msgs::PointStamped target_point, source_point;
    source_point.header = Cluster_Minpts_.header;
    source_point.point = Cluster_Minpts_.Most_closest_pt;
    
    try {
        listener.transformPoint(FRAME_ROBOT_BASE, source_point, target_point);

        // ターゲットフレームでの座標を表示
        ROS_INFO("Transformed Point: x=%f, y=%f, z=%f", target_point.point.x, target_point.point.y, target_point.point.z);
    } catch (tf::TransformException& ex) {
        ROS_ERROR("%s", ex.what());
        return;
    }

    double target_angle_pan = atan2(target_point.point.y, target_point.point.x);
    ROS_INFO("x: %f, y: %f, theta: %f",target_point.point.x, target_point.point.y, target_angle_pan);
    if (target_angle_pan < 0.87 && target_angle_pan > -0.87) 
    {
        tf::StampedTransform transform;
        try {
            // カメラフレームの姿勢を取得
            listener.lookupTransform(FRAME_CAMERA_BASE, FRAME_ROBOT_BASE, ros::Time(0), transform);
            double roll, pitch, camera_yaw;
            tf::Matrix3x3(transform.getRotation()).getRPY(roll, pitch, camera_yaw);

            std_msgs::Float64 pan_angle;
            ROS_INFO("GAIN_PAN_P: %f, delta_theta: %f, camera_yaw: %f",GAIN_PAN_P, target_angle_pan, camera_yaw);
            pan_angle.data = GAIN_PAN_P*(target_angle_pan - camera_yaw);
            pub_pan_.publish(pan_angle);
            
        } catch (tf::TransformException& ex) {
            ROS_ERROR("%s", ex.what());
        }

    }
    else
    {
        std_msgs::Float64 pan_angle;
        pan_angle.data = 0;
        pub_pan_.publish(pan_angle);
    }
    
}

