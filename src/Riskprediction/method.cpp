#include<mynteye_pointcloud/Riskprediction.h>

void RiskClass::close_point_callback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    Cluster_Minpts = *msg;
    manage();
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
    double closest_distance_;
    closest_distance_ = std::numeric_limits<double>::infinity();
    for(int k = 0; k < Cluster_Minpts.pt.size(); k++)
    {
        double distance = Cluster_risk.distance[k];
        if (distance < closest_distance_) 
        {
            closest_distance_ = distance;
        }
    }
    
}

void RiskClass::pantilt_order()
{
    
}