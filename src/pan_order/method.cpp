#include<mynteye_pointcloud/Pan_Order.h>

void OrderClass::__angle_callback(const mynteye_pointcloud::SetAngle& msg)
{
    camera_angle_ = msg;
    // __manage();
}

void OrderClass::mainloop()
{
    ros::Rate loop_rate(2);
	while (ros::ok())
	{
        __riskobject();
        __pantilt_order();
        __publish();
		ros::spinOnce();
		loop_rate.sleep();
	}
}

// void OrderClass::__manage()
// {
//     __riskobject();
//     __pantilt_order();
//     __publish();
// }

void OrderClass::__riskobject()
{
    most_Cluster_theta_ = (camera_angle_.angle)/M_PI*180;
}

void OrderClass::__pantilt_order()
{
    pubPanData_.id = camera_angle_.id;
    pubPanData_.position = int((most_Cluster_theta_*11.6) + 2048);
}

void OrderClass::__publish()
{
    pub_psition_.publish(pubPanData_);
}
