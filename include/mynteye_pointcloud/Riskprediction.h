#include <ros/ros.h>

#include <std_msgs/Float64.h>

#include <mynteye_pointcloud/pointData.h>

#include <tf/transform_listener.h>
#include <geometry_msgs/PointStamped.h>


//クラスの定義
class RiskClass{
    private:
		ros::NodeHandle nh_;
		ros::Subscriber sub_Cluster_closest_point_;
        ros::Subscriber sub_angle_;
        ros::Publisher pub_pan_, pub_tilt_;
        mynteye_pointcloud::pointData Cluster_Minpts_;

        std::string FRAME_ROBOT_BASE, FRAME_CAMERA_BASE, TOPIC_PAN_CMD, TOPIC_TILT_CMD;
        double GAIN_PAN_P = 1, GAIN_PAN_I = 0, GAIN_PAN_D = 0, GAIN_TILT_P = 1, GAIN_TILT_I = 0, GAIN_TILT_D = 0;

        void __setLaunchParam();
        void __Cluster_closest_pointcallback(const mynteye_pointcloud::pointDataConstPtr& msg);
        void __pantilt_order();
        
    public:
        RiskClass();
        ~RiskClass();
        
        void mainloop();
        
};