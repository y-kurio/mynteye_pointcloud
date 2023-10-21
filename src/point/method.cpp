#include<mynteye_pointcloud/point.h>

void pointClass::pcloud_callback(const sensor_msgs::PointCloud2ConstPtr& msg)
{   
    pcl::fromROSMsg (*msg, pcloud);
    manage();
}

void pointClass::manage()
{
    Extract();
    Clustering();
    publishPointCloud();
    clearMessages();
}

void pointClass::Clustering()
{
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(pcloud.makeShared());

    pcl::PointXYZ search_point; // 検索する点を設定
    search_point.x = 1.0;
    search_point.y = 2.0;
    search_point.z = 3.0;

    int K = 1; // 最も近い1つの点を探す

    std::vector<int> pointIdxNKNSearch(K);
    std::vector<float> pointNKNSquaredDistance(K);

    if (kdtree.nearestKSearch(search_point, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0) {
        for (size_t i = 0; i < pointIdxNKNSearch.size(); ++i) {
            int point_index = pointIdxNKNSearch[i];
            pcl::PointXYZ closest_point = pcloud.points[point_index];
            ROS_INFO("Closest point coordinates: x = %f, y = %f, z = %f", closest_point.x, closest_point.y, closest_point.z);
        }
    }

}

void pointClass::Extract()
{
    
    int count=0;
    int datanum = pcloud.height * pcloud.width;
    cloud.data.resize(datanum);
    cloud.index.resize(datanum);
    for(int i = 0 ; i < datanum; i++)
    {

        X_ = pcloud.points[i].y;   //横幅
        Y_ = pcloud.points[i].x;   //奥行
        Z_ = pcloud.points[i].z;   //高さ

        if(Z_ >= EXTRACT_ZMIN && Z_ <= EXTRACT_ZMAX)
        {
            cloud.data[count].x = X_;
            cloud.data[count].y = Y_;
            cloud.data[count].z = Z_;
            cloud.index[count].data = i;
            //std::cout<< cloud.data[count] <<std::endl;
            count++;
        
        }
        
    }
    std::cout<<"count:"<< count <<std::endl;
    cloud.data.resize(count);
    cloud.index.resize(count);

}

void pointClass::publishPointCloud()
{
    pubpcloud_.publish(cloud);
    pubrawcloud_.publish(pcloud);
}

void pointClass::clearMessages()
{
    cloud.data.clear();
    cloud.index.clear();
    pcloud.clear();
}