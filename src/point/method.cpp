#include<mynteye_pointcloud/point.h>

void pointClass::pcloud_callback(const sensor_msgs::PointCloud2ConstPtr& msg)
{   
    pcl::fromROSMsg (*msg, pcloud);
    manage();
}

void pointClass::Cluster_callback(const mynteye_pointcloud::ClassificationData::ConstPtr& msg)
{
    Clstr = *msg;
}

void pointClass::manage()
{
    Extract();
    minpt();
    // Clustering();
    publishPointCloud();
    clearMessages();
    
}

// void pointClass::Clustering()
// {
//     pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
//     kdtree.setInputCloud(pcloud.makeShared());

//     pcl::PointXYZ search_point; // 検索する点を設定
//     search_point.x = 0.0;
//     search_point.y = 0.0;
//     search_point.z = 0.0;

//     int K = 1; // 最も近い1つの点を探す

//     std::vector<int> pointIdxNKNSearch(K);
//     std::vector<float> pointNKNSquaredDistance(K);

//     if (kdtree.nearestKSearch(search_point, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0) {
//         for (size_t i = 0; i < pointIdxNKNSearch.size(); ++i) {
//             int point_index = pointIdxNKNSearch[i];
//             closest_point = pcloud.points[point_index];
//             ROS_INFO("Closest point coordinates: x = %f, y = %f, z = %f", closest_point.x, closest_point.y, closest_point.z);
//         }
//     }

// }

void pointClass::minpt()
{
    // 初期値として一番近い点の位置をゼロに設定

    // 初期値として一番近い点の距離を最大値に設定
    closest_distance_ = std::numeric_limits<double>::max();

for (int k = 0; k < Clstr.data.size(); k++)
    {
    for (int m = 0; m < Clstr.data[k].pt.size(); m++)
        {
        double x = Clstr.data[k].pt[m].x;
        double y = Clstr.data[k].pt[m].y;
        double z = Clstr.data[k].pt[m].z;
        // これまでの一番近い点との距離を計算
        double distance = std::sqrt(x * x + y * y + z * z);
                // より近い点が見つかった場合、情報を更新
                if (distance < closest_distance_) 
                {
                    closest_distance_ = distance;
                    closest_pt.close_pt[k].x = x;
                    closest_pt.close_pt[k].y = y;
                    closest_pt.close_pt[k].z = z;
                }
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
    // pubkd_.publish(closest_point);
    pubrawcloud_.publish(pcloud);
    // 一番近い点の座標をパブリッシュ
    closest_point_pub_.publish(closest_pt);
}

void pointClass::clearMessages()
{
    cloud.data.clear();
    cloud.index.clear();
    pcloud.clear();
}