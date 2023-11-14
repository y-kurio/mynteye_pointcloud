#include<mynteye_pointcloud/Riskprediction.h>

void RiskClass::__Cluster_closest_pointcallback(const mynteye_pointcloud::pointDataConstPtr& msg)
{
    Cluster_Minpts_ = *msg;
    __manage();
}

void RiskClass::__manage()
{
    // __riskobject();
    __pantilt_order();
    __publish();
}

// void RiskClass::__riskobject()
// {
//     Cluster_risk_ = Cluster_Minpts_;
//     Cluster_risk_.pt.resize(Cluster_Minpts_.pt.size());
//     Cluster_risk_.distance.resize(Cluster_Minpts_.distance.size());
//     most_Cluster_theta_ = 90-(__Cluster_ang(Cluster_Minpts_.Most_closest_pt.x, Cluster_Minpts_.Most_closest_pt.y)/3.14159265358972323*180);
//     most_Cluster_theta_ = (camera_angle_.angle)/M_PI*180;
//     // ROS_INFO("%f,   x=%f,   y=%f", most_Cluster_theta_, Cluster_Minpts_.Most_closest_pt.x, Cluster_Minpts_.Most_closest_pt.y);
// }

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
    Cluster_risk_ = Cluster_Minpts_;
    Cluster_risk_.pt.resize(Cluster_Minpts_.pt.size());
    Cluster_risk_.distance.resize(Cluster_Minpts_.distance.size());
    // ROS_INFO("syokiti: %f",cluster_delta_theta);
    double object_x = 0;
    double object_y = 0;
    ROS_INFO("pre_delta_theta: %f",delta_theta/3.14159265358972323*180);
    object_x = Cluster_Minpts_.Most_closest_pt.x*cos(-delta_theta) - Cluster_Minpts_.Most_closest_pt.y*sin(-delta_theta);
    object_y = Cluster_Minpts_.Most_closest_pt.y*cos(-delta_theta) + Cluster_Minpts_.Most_closest_pt.x*sin(-delta_theta);
    most_Cluster_theta_ = __Cluster_ang(object_x, object_y);
    if (most_Cluster_theta_ < 0.84 && most_Cluster_theta_ > -0.84) 
        {
            most_Cluster_theta_ = (__Cluster_ang(object_x, object_y)/3.14159265358972323*180);
            pubPanData_.id = 2;
            // pubPanData_.position = pan_tilt_order_;
            pubPanData_.position = int((most_Cluster_theta_*11.6) + 2048);
            delta_theta = __Cluster_ang(object_x, object_y);
        }
    else
        {
            pubPanData_.id = 2;
            // pubPanData_.position = pan_tilt_order_;
            pubPanData_.position = 2048;
            delta_theta = 0;
        }
    // else if(most_Cluster_theta_ > 0.83)
    //     {
    //         pubPanData_.position = 2602;
    //     }
    // else if(most_Cluster_theta_ < -0.83)
    //     {
    //         pubPanData_.position = 1477;
    //     }
    ROS_INFO("cur_delta_theta: %f",delta_theta/3.14159265358972323*180);
    ROS_INFO("pan_order: %d",pubPanData_.position);
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



// void RiskClass::__pantilt_order()
// {
//     static bool initialized = false;
//     static int Cur_pan_tilt_order;
//     if (!initialized) {
//         // 初回のコールバック時のみ初期値を設定
//         Cur_pan_tilt_order = 2048; // ロボットの正面の指令値
//         initialized = true;
//     }
//     Cluster_risk_ = Cluster_Minpts_;
//     Cluster_risk_.pt.resize(Cluster_Minpts_.pt.size());
//     Cluster_risk_.distance.resize(Cluster_Minpts_.distance.size());
//     double object_x, object_y;
//     most_Cluster_theta_ = 90-(__Cluster_ang(object_x, object_y)/3.14159265358972323*180);
//     // // ROS_INFO("pan_order = %d", Cur_pan_tilt_order);
//     // int pan_order_ = (most_Cluster_theta_*11.6)+Cur_pan_tilt_order;
//     // // ROS_INFO("order = %d", Cur_pan_tilt_order);
//     // // if (pan_order_ < 2602 && pan_order_ > 1477) 
//     // //     {
//     //     if(pan_order_ < 2602 && pan_order_ > 2545)
//     //     {
//     //         pan_tilt_order_ = 2602;
//     //     }
//     //     else if(pan_order_ < 2546 && pan_order_ > 2430)
//     //     {
//     //         pan_tilt_order_ = 2489;
//     //     }
//     //     else if(pan_order_ < 2431 && pan_order_ > 2315)
//     //     {
//     //         pan_tilt_order_ = 2376;
//     //     }
//     //     else if(pan_order_ < 2316 && pan_order_ > 2210)
//     //     {
//     //         pan_tilt_order_ = 2263;
//     //     }
//     //     else if(pan_order_ < 2211 && pan_order_ > 2100)
//     //     {
//     //         pan_tilt_order_ = 2150;
//     //     }
//     //     else if(pan_order_ < 2101 && pan_order_ > 1985)
//     //     {
//     //         pan_tilt_order_ = 2048;
//     //     }
//     //     else if(pan_order_ < 1986 && pan_order_ > 1875)
//     //     {
//     //         pan_tilt_order_ = 1932;
//     //     }
//     //     else if(pan_order_ < 1876 && pan_order_ > 1765)
//     //     {
//     //         pan_tilt_order_ = 1818;
//     //     }
//     //     else if(pan_order_ < 1876 && pan_order_ > 1765)
//     //     {
//     //         pan_tilt_order_ = 1705;
//     //     }
//     //     else if(pan_order_ < 1656 && pan_order_ > 1541)
//     //     {
//     //         pan_tilt_order_ = 1590;
//     //     }
//     //     else if(pan_order_ < 1542 && pan_order_ > 1477)
//     //     {
//     //         pan_tilt_order_ = 1477;
//     //     }
//     //     else
//     //     {
//     //         pan_tilt_order_ = 2048;
//     //     }
//     //         // pan_tilt_order = pan_order_;
//     //     // }
//     // // else
//     // // {
//     // //     while (pan_order_ < 2102 && pan_order_ > 1977 )
//     // //     {
//     // //         if (pan_order_ > 2048) 
//     // //             {
//     // //                 pan_tilt_order = pan_order_ - 116;
//     // //             }
//     // //         else
//     // //             {
//     // //                 pan_tilt_order = pan_order_ + 116;
//     // //             }
//     // //         pubPanData.id = 2;
//     // //         pubPanData.position = pan_tilt_order;
//     // //         pub_pan_.publish(pubPanData);
//     // //     }
//     // // }
//     // Cur_pan_tilt_order = pan_tilt_order_;


    
//     pubPanData_.id = 2;
//     // pubPanData_.position = pan_tilt_order_;
//     pubPanData_.position = int((most_Cluster_theta_*11.6) + 2048);
//     // ROS_INFO("id: %d, position: %f, angle: %f",pubPanData_.id, pubPanData_.position, (double)camera_angle_.angle);
//     // std::cout<< camera_angle_.angle << " "  << most_Cluster_theta_<< " "<< pubPanData_.position << " " << pubPanData_.id <<std::endl;
    
// }

// void RiskClass::__publish()
// {
//     pub_psition_.publish(pubPanData_);
// }

// // double RiskClass::__Cluster_ang(double& Cluster_position_x, double& Cluster_position_y){
// //     double Cluster_theta = atan2(Cluster_position_y, Cluster_position_x);
// //     return Cluster_theta;
// // }