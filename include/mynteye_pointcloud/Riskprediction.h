#include <ros/ros.h>

#include <std_msgs/Float64.h>

#include <mynteye_pointcloud/pointData.h>

#include <tf/transform_listener.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PolygonStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <string>
#include <math.h>

#include <tf2/LinearMath/Quaternion.h>

//クラスの定義
class RiskClass{
    private:
		ros::NodeHandle nh_;
		ros::Subscriber sub_Cluster_closest_point_;
        ros::Subscriber sub_angle_;
        ros::Publisher pub_pan_tilt_angle_, pub_pan_, pub_tilt_, marker_pub_;
        mynteye_pointcloud::pointData Cluster_Minpts_, Cluster_Minpts_pre_;

        std::string FRAME_ROBOT_BASE, FRAME_CAMERA_BASE, TOPIC_PAN_CMD, TOPIC_TILT_CMD;
        double OMOMI1, OMOMI2, kikenndo_sikiiti;
        double GAIN_PAN_P = 1, GAIN_PAN_I = 0, GAIN_PAN_D = 0, GAIN_TILT_P = 1, GAIN_TILT_I = 0, GAIN_TILT_D = 0;

        void __setLaunchParam();
        void __Cluster_closest_pointcallback(const mynteye_pointcloud::pointDataConstPtr& msg);
        // void __config_callback(const mynteye_pointcloud::risk_predictionConfig& config, uint32_t level)
        void __pantilt_order();
        
    public:
        RiskClass();
        ~RiskClass();
        
        void mainloop();
        
};