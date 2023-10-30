#include<mynteye_pointcloud/point.h>

// void pointClass::pcloud_callback(const sensor_msgs::PointCloud2ConstPtr& msg)
// {   
//     pcl::fromROSMsg (*msg, pcloud);
// }

void pointClass::Cluster_callback(const mynteye_pointcloud::ClassificationData::ConstPtr& msg)
{
    Clstr = *msg;
    manage();
}

void pointClass::manage()
{
    // Extract();
    if(isClstr() ){
		// ROS_INFO("creatClstrMap");
		minpt();
	}
    // publishPointCloud();
    // clearMessages();
    
}

bool pointClass::isClstr(){//curClstrのデータの有無
	if(Clstr.header.seq > 0 && (int)Clstr.data.size() > 0){
		return true;
	}
	return false;
}

void pointClass::minpt()
{
    // 初期値として一番近い点の位置をゼロに設定
    // closest_point_.x = 0.0;
    // closest_point_.y = 0.0;

    // 初期値として一番近い点の距離を最大値に設定
    closest_distance_ = 0;

for (int k = 0; k < Clstr.data.size(); k++)
    {
        for (int m = 0; m < Clstr.data[k].pt.size(); m++)
        {
            x = Clstr.data[k].pt[m].x;
            y = Clstr.data[k].pt[m].y;
            z = Clstr.data[k].pt[m].z;
            // これまでの一番近い点との距離を計算
            double distance = std::sqrt(x * x + y * y + z * z);
            // より近い点が見つかった場合、情報を更新
            if (distance < closest_distance_) 
            {
                closest_distance_ = distance;
                minx = x;
                miny = y;
                minz = z;
            }
        }
        // closest_pt.pt[k].x = minx;
        // closest_pt.pt[k].y = miny;
        // closest_pt.pt[k].z = minz;
    }
}

// void pointClass::Extract()
// {
    
//     int count=0;
//     int datanum = pcloud.height * pcloud.width;
//     cloud.data.resize(datanum);
//     cloud.index.resize(datanum);
//     for(int i = 0 ; i < datanum; i++)
//     {

//         X_ = pcloud.points[i].y;   //横幅
//         Y_ = pcloud.points[i].x;   //奥行
//         Z_ = pcloud.points[i].z;   //高さ

//         if(Z_ >= EXTRACT_ZMIN && Z_ <= EXTRACT_ZMAX)
//         {
//             cloud.data[count].x = X_;
//             cloud.data[count].y = Y_;
//             cloud.data[count].z = Z_;
//             cloud.index[count].data = i;
//             //std::cout<< cloud.data[count] <<std::endl;
//             count++;
        
//         }
        
//     }
//     std::cout<<"count:"<< count <<std::endl;
//     cloud.data.resize(count);
//     cloud.index.resize(count);

// }

// void pointClass::publishPointCloud()
// {
// //     pubpcloud_.publish(cloud);
// //     // pubkd_.publish(closest_point);
// //     pubrawcloud_.publish(pcloud);
// //     // // 一番近い点の座標をパブリッシュ
//     closest_point_pub_.publish(closest_pt);
// }

// void pointClass::clearMessages()
// {
//     cloud.data.clear();
//     cloud.index.clear();
//     pcloud.clear();
// }