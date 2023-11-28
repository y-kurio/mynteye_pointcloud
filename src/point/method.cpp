#include<mynteye_pointcloud/point.h>

void pointClass::__marker_callback(const visualization_msgs::MarkerArray::ConstPtr& msg)
{
    // ROS_INFO("__marker_callback");
    if(msg->markers.empty()) return;
    visualization_msgs::MarkerArray marker_array = *msg;
    std::vector<geometry_msgs::Point> gc_points;
    for (int i = 0; i < marker_array.markers.size(); i++)
    {
        double minX = marker_array.markers[i].points[0].x;
        double maxX = marker_array.markers[i].points[0].x;
        double minY = marker_array.markers[i].points[0].y;
        double maxY = marker_array.markers[i].points[0].y;
        double minZ = marker_array.markers[i].points[0].z;
        double maxZ = marker_array.markers[i].points[0].z;
        for (const auto& point : marker_array.markers[i].points) 
        {
            minX = std::min(minX, static_cast<double>(point.x));
            minY = std::min(minY, static_cast<double>(point.y));
            minZ = std::min(minZ, static_cast<double>(point.z));
            maxX = std::max(maxX, static_cast<double>(point.x));
            maxY = std::max(maxY, static_cast<double>(point.y));
            maxZ = std::max(maxZ, static_cast<double>(point.z));
        }
        geometry_msgs::Point gc;
        gc.x = (maxX - minX)/2 + minX;
        gc.y = (maxY - minY)/2 + minY;
        gc.z = (maxZ - minZ)/2 + minZ;
        gc_points.push_back(gc);
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