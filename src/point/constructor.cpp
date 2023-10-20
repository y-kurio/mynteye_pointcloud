#include<mynteye_pointcloud/point.h>

pointClass::pointClass()
{
    //subscriber
	sub_point_=nh.subscribe("/mynteye/points/data_raw",1,&pointClass::pcloud_callback,this);
    pubpcloud_= nh.advertise<mynteye_pointcloud::PointCloudData>("/pointcloud", 1);
	pubrawcloud_= nh.advertise<sensor_msgs::PointCloud2>("/pcloud", 1);
	// lanchファイルの読み込み
	setLaunchParam();
}       
pointClass::~pointClass()
{

}