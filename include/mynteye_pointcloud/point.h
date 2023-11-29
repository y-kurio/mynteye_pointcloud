#include <ros/ros.h>

#include <mynteye_pointcloud/pointData.h>

#include <mynteye_pointcloud/PointCloudData.h>
#include <visualization_msgs/MarkerArray.h>

//クラスの定義
class pointClass{
    private:
		ros::NodeHandle nh_;
		ros::Subscriber marker_sub_;
        mynteye_pointcloud::pointData Cluster_closest_pt_;
        ros::Publisher closest_point_pub_;

        void __marker_callback(const visualization_msgs::MarkerArray::ConstPtr& msg);
        void __setLaunchParam();
        void __publishPointCloud();
    public:
        pointClass();
        ~pointClass();
        
        void mainloop();
};