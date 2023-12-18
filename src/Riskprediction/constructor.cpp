#include<mynteye_pointcloud/Riskprediction.h>

RiskClass::RiskClass()
{
	__setLaunchParam();
	sub_Cluster_closest_point_	= nh_.subscribe("Cluster_closest_point",1,&RiskClass::__Cluster_closest_pointcallback,this);
	marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("marker", 1);
	pub_pan_					= nh_.advertise<std_msgs::Float64>(TOPIC_PAN_CMD, 1);
	pub_tilt_					= nh_.advertise<std_msgs::Float64>(TOPIC_TILT_CMD, 1);
	pub_pan_tilt_angle_	= nh_.advertise<geometry_msgs::PoseStamped>("/pantilt_transform", 1);
}
RiskClass::~RiskClass()
{

}