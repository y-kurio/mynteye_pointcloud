#include<mynteye_pointcloud/Pan_Order.h>

OrderClass::OrderClass()
{
	sub_angle_ =nh_.subscribe("set_angle",1,&OrderClass::__angle_callback,this);
    pub_psition_=nh_.advertise<dynamixel_sdk_examples::SetPosition>("set_position",1);
	__setLaunchParam();
}
OrderClass::~OrderClass()
{

}