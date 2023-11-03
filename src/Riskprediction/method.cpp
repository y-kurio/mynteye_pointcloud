#include<mynteye_pointcloud/Riskprediction.h>

void RiskClass::Cluster_closest_pointcallback(const mynteye_pointcloud::pointDataConstPtr& msg)
{
    Cluster_Minpts = *msg;
    manage();

    // 以降の処理で value を使用
}

void RiskClass::manage()
{
    riskobject();
    pantilt_order();
    publish();
}

void RiskClass::riskobject()
{
    Cluster_risk = Cluster_Minpts;
    Cluster_risk.pt.resize(Cluster_Minpts.pt.size());
    Cluster_risk.distance.resize(Cluster_Minpts.distance.size());
    // double closest_distance_;      //ittannkanngaeru
    // closest_distance_ = std::numeric_limits<double>::infinity();
    // for(int k = 0; k < Cluster_Minpts.pt.size(); k++)
    // {
    //     double distance = Cluster_risk.distance[k].data;
    //     if (distance < closest_distance_) 
    //     {
    //         closest_distance_ = distance;
    //     }
    // }
    most_Cluster_theta = 90-(Cluster_ang(Cluster_Minpts.Most_closest_pt.x, Cluster_Minpts.Most_closest_pt.y)/3.14159265358972323*180);
    // ROS_INFO("%f", most_Cluster_theta);
}

void RiskClass::pantilt_order()
{
    static bool initialized = false;
    static int Cur_pan_tilt_order;

    if (!initialized) {
        // 初回のコールバック時のみ初期値を設定
        Cur_pan_tilt_order = 2048; // 例として 42.0 という初期値を設定
        initialized = true;
    }
    int pan_order_ = int((most_Cluster_theta*11.6)+Cur_pan_tilt_order);
    if (pan_order_ < 2602 && pan_order_ > 1477) 
            {
                pan_tilt_order = pan_order_;
                ROS_INFO("order = %f", pan_tilt_order);
            }
    pubPanData.id = 2;
    pubPanData.position = pan_tilt_order;
    Cur_pan_tilt_order = pan_tilt_order;
}

void RiskClass::publish()
{
    pub_pan_.publish(pubPanData);
}

double RiskClass::Cluster_ang(double& Cluster_position_x, double& Cluster_position_y){
    double Cluster_theta = atan2(Cluster_position_y, Cluster_position_x);
    return Cluster_theta;
}