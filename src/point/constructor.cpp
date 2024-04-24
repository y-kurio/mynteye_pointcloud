#include<mynteye_pointcloud/point.h>

pointClass::pointClass()
{
	__setLaunchParam();
	marker_sub_			= nh_.subscribe("robot_4/potbot/filtered_pcl", 1, &pointClass::__marker_callback, this);
	closest_point_pub_	= nh_.advertise<mynteye_pointcloud::pointData>("Cluster_closest_point", 1);
	
}       
pointClass::~pointClass()
{

}