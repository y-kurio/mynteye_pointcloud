#include<mynteye_pointcloud/point.h>

pointClass::pointClass()
{
    //subscriber
	// sub_point_=nh.subscribe("/mynteye/points/data_raw",1,&pointClass::pcloud_callback,this);
    // pubpcloud_= nh.advertise<mynteye_pointcloud::PointCloudData>("/pointcloud", 1);
	// pubrawcloud_= nh.advertise<sensor_msgs::PointCloud2>("/pcloud", 1);
	// ポイントデータの購読者を作成
	point_sub_ = nh.subscribe("/classificationData", 1, &pointClass::Cluster_callback, this);
	// 一番近い点の位置をパブリッシュするためのパブリッシャを作成
	// closest_point_pub_ = nh.advertise<mynteye_pointcloud::pointData>("/closest_point", 1);
	// // pubkd_= nh.advertise<sensor_msgs::PointCloud2>("/kd", 1);
	// lanchファイルの読み込み
	setLaunchParam();
}       
pointClass::~pointClass()
{

}