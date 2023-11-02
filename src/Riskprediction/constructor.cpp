#include<mynteye_pointcloud/Riskprediction.h>

RiskClass::RiskClass()
{
    //subscriber
	sub_close_point_ =nh.subscribe("closest_point",1,&RiskClass::close_point_callback,this);
	// Risk_pub_ = nh.subscribe("Riskobject", 1, &RiskClass::Cluster_callback, this);
    pub_pan_=nh_pub.advertise<dynamixel_sdk_examples::SetPosition>("set_position",1);
	pub_tilt_=nh_pub.advertise<dynamixel_sdk_examples::SetPosition>("set_position",1);
	// lanchファイルの読み込み
	setLaunchParam();
}
RiskClass::~RiskClass()
{

}