#include<mynteye_pointcloud/Riskprediction.h>

RiskClass::RiskClass()
{
    //subscriber
	sub_Cluster_closest_point_ =nh_.subscribe("Cluster_closest_point",1,&RiskClass::__Cluster_closest_pointcallback,this);
	// sub_angle_ =nh_.subscribe("set_angle",1,&RiskClass::__angle_callback,this);
	// sub_most_Cluster_closest_point =nh.subscribe("Cluster_closest_point",1,&RiskClass::most_Cluster_closest_pointcallback,this);
	// Risk_pub_ = nh.subscribe("Riskobject", 1, &RiskClass::Cluster_callback, this);
    pub_psition_=nh_.advertise<dynamixel_sdk_examples::SetPosition>("set_position",1);
	// pub_tilt_=nh.advertise<dynamixel_sdk_examples::SetPosition>("set_position",1);
	// lanchファイルの読み込み
	__setLaunchParam();
}
RiskClass::~RiskClass()
{

}