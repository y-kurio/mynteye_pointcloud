#include<mynteye_pointcloud/tf_pantilt.h>

//このプログラムは受け取ったデータを外部に送信します。
int main(int argc,char **argv){
	ros::init(argc,argv,"tf_pantilt_br");
    BroadCaster br;
    ros::spin();

    // ros::NodeHandle nh;
    // tf2_ros::TransformBroadcaster dynamic_br;

    // while(ros::ok())
    // {
    //     ros::Time rostime = ros::Time::now();
    //     double t = rostime.toSec();
    //     geometry_msgs::TransformStamped transformStamped_pan;
    //     transformStamped_pan.header.stamp = rostime;
    //     transformStamped_pan.header.frame_id = "robot4/camera_stay_link";
    //     transformStamped_pan.child_frame_id = "robot4/pan_link";
    //     transformStamped_pan.transform.translation.x = 0.0;
    //     transformStamped_pan.transform.translation.y = 0.0;
    //     transformStamped_pan.transform.translation.z = 0.5;
    //     double pan = sin(0.1*t);
    //     tf2::Quaternion q;
    //     q.setRPY(0, 0, pan);
    //     transformStamped_pan.transform.rotation.x = q.x();
    //     transformStamped_pan.transform.rotation.y = q.y();
    //     transformStamped_pan.transform.rotation.z = q.z();
    //     transformStamped_pan.transform.rotation.w = q.w();
    //     dynamic_br.sendTransform(transformStamped_pan);

    //     geometry_msgs::TransformStamped transformStamped_tilt;
    //     transformStamped_tilt.header.stamp = rostime;
    //     transformStamped_tilt.header.frame_id = "robot4/pan_link";
    //     transformStamped_tilt.child_frame_id = "robot4/tilt_link";
    //     transformStamped_tilt.transform.translation.x = 0.05;
    //     transformStamped_tilt.transform.translation.y = -0.1;
    //     transformStamped_tilt.transform.translation.z = 0.0;
    //     tf2::Quaternion quat;
    //     double tilt = cos(0.15*t);
    //     quat.setRPY(0.0, tilt, 0.0);
    //     transformStamped_tilt.transform.rotation.x = quat.x();
    //     transformStamped_tilt.transform.rotation.y = quat.y();
    //     transformStamped_tilt.transform.rotation.z = quat.z();
    //     transformStamped_tilt.transform.rotation.w = quat.w();
    //     dynamic_br.sendTransform(transformStamped_tilt);
    // }

    // // rp.mainloop();
    // return 0;

	return 0;
}