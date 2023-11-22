#include<mynteye_pointcloud/Riskprediction.h>

void RiskClass::__Cluster_closest_pointcallback(const mynteye_pointcloud::pointDataConstPtr& msg)
{
    Cluster_Minpts_ = *msg;
    __manage();
}

// void RiskClass::mainloop()
// {
//     ros::Rate loop_rate(2);
// 	while (ros::ok())
// 	{
//         // __riskobject();
//         __pantilt_order();
//         __publish();
// 		ros::spinOnce();
// 		loop_rate.sleep();
// 	}
// }

void RiskClass::__manage()
{
    // __riskobject();
    __pantilt_order();
    __publish();
}

void RiskClass::__riskobject()
{
    double object_x, object_y;
    static bool initialized = false;
    static double delta_theta;
    static double kikenndo;
    // static mynteye_pointcloud::pointData  Cluster_risk_.pt;
    if (!initialized) {
        delta_theta = 0;
        kikenndo = 0;
        // Cluster_risk_.pt = 0;
        initialized = true;
    }
    for (int k = 0; k < Cluster_Minpts_.pt.size(); k++)
    {
        object_x = Cluster_Minpts_.pt[k].x*cos(-delta_theta) - Cluster_Minpts_.pt[k].y*sin(-delta_theta);
        object_y = Cluster_Minpts_.pt[k].y*cos(-delta_theta) + Cluster_Minpts_.pt[k].x*sin(-delta_theta);
        Cluster_risk_.distance[k].data = std::sqrt(object_x * object_x + object_y * object_y)-std::sqrt(Cluster_risk_.pt[k].x * Cluster_risk_.pt[k].x + Cluster_risk_.pt[k].y * Cluster_risk_.pt[k].y);
        Cluster_risk_.pt[k].x = Cluster_Minpts_.pt[k].x*cos(-delta_theta) - Cluster_Minpts_.pt[k].y*sin(-delta_theta);
        Cluster_risk_.pt[k].y = Cluster_Minpts_.pt[k].x*cos(-delta_theta) - Cluster_Minpts_.pt[k].y*sin(-delta_theta);
    ROS_INFO("%d, %f", k, Cluster_risk_.distance[k].data);
    }
    //     //kamerakakudohoukouketteisiki
    // f = teisuu - teisuu(0-delta_theta) + teisuu(kikenndo * std::exp())
}

void RiskClass::__pantilt_order()
{
    static bool initialized = false;
    static double delta_theta;
    // static double cluster_delta_theta;
    // cluster_delta_theta = __Cluster_ang(Cluster_Minpts_.Most_closest_pt.x, Cluster_Minpts_.Most_closest_pt.y);
    if (!initialized) {
        // 初回のコールバック時のみ初期値を設定
        delta_theta = 0; // ロボットの正面の指令値
        // cluster_delta_theta = 0;
        initialized = true;
    }
    double object_x = 0;
    double object_y = 0;
    object_x = Cluster_Minpts_.Most_closest_pt.x;//*sin(delta_theta) - Cluster_Minpts_.Most_closest_pt.y*cos(delta_theta);
    object_y = Cluster_Minpts_.Most_closest_pt.y;//*cos(delta_theta) + Cluster_Minpts_.Most_closest_pt.x*sin(delta_theta);
    // ROS_INFO("robo_x: %f, robo_y: %f, cam_x: %f, cam_y: %f, theta: %f",object_x, object_y, Cluster_Minpts_.Most_closest_pt.x, Cluster_Minpts_.Most_closest_pt.y);
    most_Cluster_theta_ = __Cluster_ang(object_x, object_y) + delta_theta;
    ROS_INFO("heikin_x: %f, heikin_y: %f",Cluster_Minpts_.Most_closest_pt.x, Cluster_Minpts_.Most_closest_pt.y);
    if (most_Cluster_theta_ < 0.87 && most_Cluster_theta_ > -0.87) 
        {
            // int pan_order_ = ((most_Cluster_theta_/3.14159265358972323*180)*11.6)+2048;
            delta_theta = most_Cluster_theta_;
            // if(pan_order_ < 2602 && pan_order_ > 2545)
            // {
            //     delta_theta = 0.872;
            // }
            // else if(pan_order_ < 2546 && pan_order_ > 2430)
            // {
            //     delta_theta = 0.698;
            // }
            // else if(pan_order_ < 2431 && pan_order_ > 2315)
            // {
            //     delta_theta = 0.523;
            // }
            // else if(pan_order_ < 2316 && pan_order_ > 2210)
            // {
            //     delta_theta = 0.349;
            // }
            // else if(pan_order_ < 2211 && pan_order_ > 2100)
            // {
            //     delta_theta = 0.175;
            // }
            // else if(pan_order_ < 2101 && pan_order_ > 1985)
            // {
            //     delta_theta = 0;
            // }
            // else if(pan_order_ < 1986 && pan_order_ > 1875)
            // {
            //     delta_theta = -0.175;
            // }
            // else if(pan_order_ < 1876 && pan_order_ > 1765)
            // {
            //     delta_theta = -0.349;
            // }
            // else if(pan_order_ < 1876 && pan_order_ > 1765)
            // {
            //     delta_theta = -0.523;
            // }
            // else if(pan_order_ < 1656 && pan_order_ > 1541)
            // {
            //     delta_theta = -0.698;
            // }
            // else if(pan_order_ < 1542 && pan_order_ > 1477)
            // {
            //     delta_theta = -0.872;
            // }
            // most_Cluster_theta_ = (__Cluster_ang(object_x, object_y)/3.14159265358972323*180);
            pubPanData_.id = 2;
            // pubPanData_.position = pan_tilt_order_;
            // pubPanData_.position = int((most_Cluster_theta_*11.6) + 2048);
            pubPanData_.angle = delta_theta;
        }
    else
        {
            pubPanData_.id = 2;
            // pubPanData_.position = pan_tilt_order_;
            // pubPanData_.position = 2048;
            pubPanData_.angle = 0;
            delta_theta = 0;
        }
        // ROS_INFO("pan_order_seisuu: %f",((delta_theta/3.14159265358972323*180)*11.6)+2048);
    // else if(most_Cluster_theta_ > 0.83)
    //     {
    //         pubPanData_.position = 2602;
    //     }
    // else if(most_Cluster_theta_ < -0.83)
    //     {
    //         pubPanData_.position = 1477;
    //     }
    // ROS_INFO("cur_delta_theta: %f",delta_theta/3.14159265358972323*180);
    // ROS_INFO("pan_order: %f",pubPanData_.angle);
    // ROS_INFO("id: %d, position: %f, angle: %f",pubPanData_.id, pubPanData_.position, (double)camera_angle_.angle);
    // std::cout<< camera_angle_.angle << " "  << most_Cluster_theta_<< " "<< pubPanData_.position << " " << pubPanData_.id <<std::endl;
    
}

void RiskClass::__publish()
{
    pub_psition_.publish(pubPanData_);
}

double RiskClass::__Cluster_ang(double& Cluster_position_x, double& Cluster_position_y){
    double Cluster_theta = atan2(Cluster_position_x, Cluster_position_y);
    return Cluster_theta;
}


