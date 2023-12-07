#include<mynteye_pointcloud/point.h>

void pointClass::__marker_callback(const visualization_msgs::MarkerArray::ConstPtr& msg)
{
    // ROS_INFO("__marker_callback");
    if(msg->markers.empty()) return;// 中身が空だった場合やり直し
    visualization_msgs::MarkerArray marker_array = *msg;
    std::vector<geometry_msgs::Point> gc_points;
    for (int i = 0; i < marker_array.markers.size(); i++)
    {
        if(marker_array.markers[i].ns.find("centor") != std::string::npos)
        {
            gc_points.push_back(marker_array.markers[i].pose.position);// gc_pointsに上書きしている
        }
    }
    geometry_msgs::Point gc_closest;
    double minD = sqrt(pow(gc_points[0].x,2)+pow(gc_points[0].y,2)+pow(gc_points[0].z,2));
    gc_closest = gc_points[0];
    std::vector<std_msgs::Float64> distance;
    for (const auto& gc : gc_points) 
    {
        double D = sqrt(pow(gc.x,2)+pow(gc.y,2)+pow(gc.z,2));
        std_msgs::Float64 d_tmp;
        d_tmp.data = D;
        distance.push_back(d_tmp);
        if(D < minD)
        {
            minD = D;
            gc_closest = gc;
        }
    }

    ROS_INFO("Closest Point: x=%f, y=%f, z=%f", gc_closest.x, gc_closest.y, gc_closest.z);

    Cluster_closest_pt_.header = marker_array.markers[0].header;
    Cluster_closest_pt_.distance = distance;
    Cluster_closest_pt_.pt = gc_points;
    Cluster_closest_pt_.Most_closest_pt = gc_closest;

    __publishPointCloud();

}

void pointClass::mainloop()
{
    ros::spin();    
}

void pointClass::__publishPointCloud()
{
    closest_point_pub_.publish(Cluster_closest_pt_);
}