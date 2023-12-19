#include<mynteye_pointcloud/tf_pantilt.h>

BroadCaster::BroadCaster()
{
	__setLaunchParam();
	sub_pan_angle_	= nh_.subscribe("/pantilt_transform",1,&BroadCaster::__pan_anglecallback,this);
	pan_angle_.pose.orientation = potbot_lib::utility::get_Quat(0,0,0);
}
BroadCaster::~BroadCaster()
{

}