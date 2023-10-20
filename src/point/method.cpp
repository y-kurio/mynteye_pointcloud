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