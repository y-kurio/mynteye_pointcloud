#include<mynteye_pointcloud/Riskprediction.h>
void RiskClass::__Cluster_closest_pointcallback(const mynteye_pointcloud::pointDataConstPtr& msg)
{
    Cluster_Minpts_pre_ = Cluster_Minpts_;
    Cluster_Minpts_ = *msg;
    // __pantilt_order();
}

// void RiskClass::__config_callback(const mynteye_pointcloud::risk_predictionConfig& config, uint32_t level)
// {
//     weight_kyori =config.weight_kyori;
//     weight_yokohaba = config.weight_yokohaba;
//     ROS_INFO("Reconfigure Request: %f, %f", config.weight_kyori, config.weight_yokohaba);
// }

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
    std::vector<std_msgs::Float64> risk_power;
    static tf::TransformListener listener;
    geometry_msgs::PointStamped target_point, source_point;
    std::vector<geometry_msgs::Point> transformed_points;
    std::vector<std_msgs::Float64> distance;
    std_msgs::Header source_header = Cluster_Minpts_.header;
    double minD = std::numeric_limits<double>::infinity();
    double risk_syokiti = 0;
    double most_risk;
    std_msgs::Float64 risk_tmp, close_distance;
    visualization_msgs::MarkerArray marker;
    std_msgs::Float64 pan_angle;
    geometry_msgs::PoseStamped pantilt_angle;
    for(const auto& sp : Cluster_Minpts_.pt)
    {
        source_point.header = source_header;
        source_point.point = sp;
        // ROS_INFO("Transformed Point: x=%f, y=%f, z=%f", source_point.point.x, source_point.point.y, source_point.point.z);

        try
        {
            listener.transformPoint(FRAME_ROBOT_BASE, source_point, target_point);

        // ターゲットフレームでの座標を表示
        // ROS_INFO("Transformed Point: x=%f, y=%f, z=%f", target_point.point.x, target_point.point.y, target_point.point.z);
        } catch (tf::TransformException& ex) {
            ROS_ERROR("%s", ex.what());
            return;
        }

        transformed_points.push_back(target_point.point);
    }
    for (int i = 0; i < transformed_points.size(); i++)
    {
        ROS_INFO("Transformed Point:i=%d, x=%f, y=%f, z=%f", i, transformed_points[i].x, transformed_points[i].y, transformed_points[i].z);
        double D = sqrt(pow(transformed_points[i].x,2)+pow(transformed_points[i].y,2)+pow(transformed_points[i].z,2));
        std_msgs::Float64 d_tmp;
        d_tmp.data = D;
        distance.push_back(d_tmp);
        // double D = distance[i].data;
        if(D < minD)
        {
            minD = D;
            close_distance.data = d_tmp.data;
        }
    }
    for (int j = 0; j < transformed_points.size(); j++)
    {
        // double Dis = (OMOMI1*fabs(0.5 / transformed_points[j].y) + OMOMI2*(close_distance.data / distance[j].data));
        double Dis = (OMOMI1*(-pow(transformed_points[j].y,2)+1) + OMOMI2*(close_distance.data / distance[j].data));
        risk_tmp.data = Dis;
        risk_power.push_back(risk_tmp);
        if(risk_power[j].data > risk_syokiti)
        {
            if (risk_power[j].data > 3)
            {
                risk_syokiti = risk_power[j].data;
                target_point.point.x = transformed_points[j].x;
                target_point.point.y = transformed_points[j].y;
            }else if(risk_power[j].data <= 3)
            {
                risk_syokiti = risk_power[j].data;
                target_point.point.x = abs(transformed_points[j].x);
                target_point.point.y = 0;
            }
           
        }
        most_risk = risk_syokiti;
    ROS_INFO(":i=%d, risk_power=%f, distance=%f, most_distance=%f", j, risk_power[j].data, distance[j].data, close_distance.data);
    }
    
    for (int s = 0; s < transformed_points.size(); s++)
    {
        visualization_msgs::Marker mk;

        mk.header = target_point.header;
        mk.ns = "riskmarker";
        mk.id = s;

        mk.type = visualization_msgs::Marker::CUBE;
        mk.action = visualization_msgs::Marker::ADD;
        mk.lifetime = ros::Duration();

        mk.scale.x = 0.5;
        mk.scale.y = 0.5;
        mk.scale.z = 0.2;
        mk.pose.position.x = transformed_points[s].x;
        mk.pose.position.y = transformed_points[s].y;
        mk.pose.position.z = 0;
        mk.pose.orientation.x = 0;
        mk.pose.orientation.y = 0;
        mk.pose.orientation.z = 0;
        mk.pose.orientation.w = 1;

        ROS_INFO("syokiti=%d, risk_power=%f", s, risk_power[s].data);
        if (risk_power[s].data == most_risk){
            mk.color.r = 1.0f;
            mk.color.g = 0.0f;
            mk.color.b = 0.0f;
            mk.color.a = 1.0f;
        }else if (risk_power[s].data < kikenndo_sikiiti){
            mk.color.r = 0.0f;
            mk.color.g = 0.0f;
            mk.color.b = 1.0f;
            mk.color.a = 1.0f;
        }else if (risk_power[s].data > kikenndo_sikiiti){
            mk.color.r = 0.0f;
            mk.color.g = 1.0f;
            mk.color.b = 0.0f;
            mk.color.a = 1.0f;
        }
        marker.markers.push_back(mk);
    }
    marker_pub_.publish(marker);

    double target_angle_pan = atan2(target_point.point.y, target_point.point.x);
    // ROS_INFO("x: %f, y: %f, theta: %f",target_point.point.x, target_point.point.y, target_angle_pan);
    if (target_angle_pan < 0.87 && target_angle_pan > -0.87) 
    {
        tf::StampedTransform transform;
        try {
            // カメラフレームの姿勢を取得
            listener.lookupTransform(FRAME_CAMERA_BASE, FRAME_ROBOT_BASE, ros::Time(0), transform);
            double roll, pitch, camera_yaw;
            tf::Matrix3x3(transform.getRotation()).getRPY(roll, pitch, camera_yaw);
            // ROS_INFO("GAIN_PAN_P: %f, delta_theta: %f, camera_yaw: %f",GAIN_PAN_P, target_angle_pan, camera_yaw);
            pan_angle.data = GAIN_PAN_P*(target_angle_pan - camera_yaw);
            pub_pan_.publish(pan_angle);
            
        } catch (tf::TransformException& ex) {
            ROS_ERROR("%s", ex.what());
        }
        

    }
    else
    {
        pan_angle.data = 0;
        pub_pan_.publish(pan_angle);
    }
    tf2::Quaternion quaternion;
    quaternion.setRPY(0.0, 0.0, pan_angle.data); // RPY順にセット
    // クォータニオンを出力
    std::cout << "Quaternion:" << std::endl;
    std::cout << "x: " << quaternion.x() << std::endl;
    std::cout << "y: " << quaternion.y() << std::endl;
    std::cout << "z: " << quaternion.z() << std::endl;
    std::cout << "w: " << quaternion.w() << std::endl;
    pantilt_angle.header.stamp = source_point.header.stamp;
    pantilt_angle.header.frame_id = FRAME_ROBOT_BASE;
    pantilt_angle.pose.orientation.x = quaternion.x();
    pantilt_angle.pose.orientation.y = quaternion.y();
    pantilt_angle.pose.orientation.z = quaternion.z();
    pantilt_angle.pose.orientation.w = quaternion.w();
    pub_pan_tilt_angle_.publish(pantilt_angle);
}