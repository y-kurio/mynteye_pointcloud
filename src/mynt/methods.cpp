#include<mynteye_pointcloud/PointCloud.h>

void PointCloudClass::pcl_callback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    pcl::fromROSMsg (*msg, rawcloud);
    //rawPC2=PC2=msg;
    manage();
}

void PointCloudClass::odom_callback(const nav_msgs::Odometry& msg)
{
    odom = msg;
}

// void PointCloudClass::encoder_callback(const beego_control::beego_encoder& msg)
// {
// 	if (!encoder_firsttime){
// 		encoder_time_pre = ros::Time::now();
// 		encoder_firsttime = true;
// 	}else{
// 		ros::Time encoder_time_now = ros::Time::now();
// 		double encoder_deltatime = encoder_time_now.toSec() - encoder_time_pre.toSec();
// 		//std::cout<< encoder_deltatime <<std::endl;
// 		double robot_vel = (-msg.vel.r + msg.vel.l) / 2.0;
// 		distance_traveled_robot = distance_traveled_robot + robot_vel * encoder_deltatime;
// 		//std::cout<< distance_traveled_robot <<std::endl;
// 		encoder_time_pre = encoder_time_now;
// 	}
//     //std::cout<< msg.vel <<std::endl;
// }

void PointCloudClass::depthimage_callback(const sensor_msgs::Image& msg)
{
    try{
        // bridgeImage=cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::TYPE_32FC1);
        bridgeImage=cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::MONO16);//CV_16UC1
    }
    catch(cv_bridge::Exception& e) {//エラー処理
        std::cout<<"sensor_depth_image_callback Error \n";
        ROS_ERROR("Could not convert from '%s' to 'MONO16'.",
        msg.encoding.c_str());
        return ;
    }

    int imageheight = bridgeImage->image.rows;
    int imagewidth = bridgeImage->image.cols;
    int datanum = imageheight * imagewidth;
    cloudfromimage.data.resize(datanum);
    int cnt = 0;
    for (int imgrow = 0;imgrow < imageheight ; imgrow++)
    {
        for (int imgcol = 0;imgcol < imagewidth ; imgcol++)
        {
            unsigned short int depth = bridgeImage->image.at<unsigned short int>(imgrow,imgcol);
            if (depth > 0)
            {
                cloudfromimage.data[cnt].y = depth / 1000.0;
                cloudfromimage.data[cnt].z = (-(float)imgrow + (float)imageheight) * cloudfromimage.data[cnt].y / PIXEL_TO_XYZ;//高さ算出,370.985,0.0021
                cloudfromimage.data[cnt].x = -((float)imgcol - (float)imagewidth / 2.0) * cloudfromimage.data[cnt].y / PIXEL_TO_XYZ;//0.0021/0.000006=350.0

                cnt++;
            }
            
        }
    }
    cloudfromimage.data.resize(cnt);
    pubpc1.publish(cloudfromimage);
    cloudfromimage.data.clear();

}

//manage method
void PointCloudClass::manage(){
    Extract();
    //Clustering();
    publishPointCloud();
    clearMessages();
}

void PointCloudClass::Extract()
{

    int cnt=0;
    float X,Y,Z;

    int datanum = rawcloud.height * rawcloud.width;
    cloud.data.resize(datanum);
    cloud.index.resize(datanum);
    for(int i = 0 ; i < datanum; i++)
    {

        X = rawcloud.points[i].y;   //横幅
        Y = rawcloud.points[i].x;   //奥行
        Z = rawcloud.points[i].z;   //高さ

        if((Y > EXTRACT_YMIN && Y <= EXTRACT_YMAX) && (X >= EXTRACT_XMIN && X <= EXTRACT_XMAX))//kamera.node-tawotoruhanniwositeisitesiborikonndeiru
        {
            cloud.data[cnt].x = X;
            cloud.data[cnt].y = Y;
            cloud.data[cnt].z = Z;
            cloud.index[cnt].data = i;
            //std::cout<< cloud.data[cnt] <<std::endl;
            cnt++;
        
        }
        
    }
    std::cout<<"cnt:"<< cnt <<std::endl;
    cloud.data.resize(cnt);
    cloud.index.resize(cnt);

}

bool isExist(std::vector<uint32_t> indexarr, uint32_t size, uint32_t index)
{
    for(uint32_t i = 0; i < size; i++)
    {
        if (indexarr[i] == index) return true;
    }
    return false;
}

void PointCloudClass::Clustering()
{
    int index_cluster = 0;
    int index_cluster_cloud = 0;
    int index_center = 0;

    float X,Y;
    float cX,cY;

    int datanum = cloud.data.size();
    int datanum_raw = 752*480;
    bool added[datanum_raw];
    bool centered[datanum_raw];
    for(int i = 0; i < datanum_raw; i++)
    {
        added[i] = false;
        centered[i] = false;
    }
    int added_index[datanum];
    int added_index_cnt = 0;
    cluster.data.resize(datanum);

    bool break_flag = false;

    int index_center_pre = -1;
    int index_cluster_pre = -1;
    
    bool flag_dup = false;

    while(true)
    {
        for (int i=0;i<datanum;i++)
        {
            
            if(!added[cloud.index[i].data])
            {
                if (index_center == index_center_pre) index_center = cloud.index[i].data;
                break;
            }

            if (i>=datanum-1) break_flag = true;
        }
        if(break_flag)break;

        //std::cout<< index_center <<std::endl;
        
        if(index_cluster != index_cluster_pre)
        {
            cluster.data[index_cluster].data.resize(datanum);
            cluster.data[index_cluster].index.resize(datanum);
        }
        index_cluster_pre = index_cluster;

        cluster.data[index_cluster].data[index_cluster_cloud] = cloud.data[index_center];
        cluster.data[index_cluster].index[index_cluster_cloud] = cloud.index[index_center];
        index_center_pre = index_center;
        added[index_center] = true;
        centered[index_center] = true;

        if (!flag_dup)
        {
            added_index[added_index_cnt] = index_center;
        }

        cX = cluster.data[index_cluster].data[index_cluster_cloud].x;
        cY = cluster.data[index_cluster].data[index_cluster_cloud].y;

        index_cluster_cloud++;

        int add_num = 0;
        for(int index_cloud = 0; index_cloud < datanum; index_cloud++)
        {
            if(added[cloud.index[index_cloud].data])
            {
                //std::cout<< index_cloud <<std::endl;
                continue;
            }
            X = cloud.data[index_cloud].x;
            Y = cloud.data[index_cloud].y;
            
            double distance = sqrt(pow((X-cX),2.0) + pow((Y-cY),2.0));
            if(distance <= 0.2)
            {

                // uint32_t size_index = cluster.data[index_cluster].index.size();
                // std::vector<uint32_t> index_vec;
                // index_vec.resize(size_index);
                // for(int i = 0; i < size_index; i++)
                // {
                //     index_vec[i] = cluster.data[index_cluster].index[i].data;
                // }

                // if(!isExist(index_vec, size_index, cloud.index[index_cloud].data))
                // {
                cluster.data[index_cluster].data[index_cluster_cloud] = cloud.data[index_cloud];
                cluster.data[index_cluster].index[index_cluster_cloud] = cloud.index[index_cloud];
                added[cloud.index[index_cloud].data] = true;
                added_index[added_index_cnt] = cloud.index[index_cloud].data;
                added_index_cnt++;
                index_cluster_cloud++;
                add_num++;
                //}
            }
            
        }
        //if(index_cluster>0)std::cout<< index_cluster_cloud <<std::endl;
        bool flag_transition_cluster = true;

        flag_dup = false;
        for(int i = 0; i < added_index_cnt; i++)
        {
            if(!centered[added_index[i]])
            {
                index_center = added_index[i];
                flag_transition_cluster = false;
                flag_dup = true;
                break;
            }
        }
        
        //if (flag)std::cout<< flag <<std::endl;
        if(flag_transition_cluster && add_num == 0)
        {
            //std::cout<< index_cluster <<std::endl;
            cluster.data[index_cluster].data.resize(index_cluster_cloud);
            cluster.data[index_cluster].index.resize(index_cluster_cloud);
            index_cluster++;
            index_cluster_cloud = 0;
            int added_index[datanum];
            added_index_cnt = 0;
        }
    }
    std::cout<<"index_cluster:"<< index_cluster <<std::endl;
    cluster.data.resize(index_cluster);

    ////重複している点を削除////
    cluster_nondup.data.resize(index_cluster);
    for (int i = 0;i < index_cluster;i++)
    {
        int cnt_nondup = 0;
        int size = cluster.data[i].data.size();
        cluster_nondup.data[i].data.resize(size);
        for (int j = 0; j < size; j++)
        {
            bool flag_nondup = true;
            geometry_msgs::Point32 comp = cluster.data[i].data[j];
            for (int k = 0; k < size; k++)
            {
                if (cluster.data[i].data[k].x == comp.x && 
                    cluster.data[i].data[k].y == comp.y && 
                    cluster.data[i].data[k].z == comp.z)
                {
                    for(int l = 0; l < cnt_nondup; l++)
                    {
                        if (cluster.data[i].data[l].x == comp.x && 
                            cluster.data[i].data[l].y == comp.y && 
                            cluster.data[i].data[l].z == comp.z)
                        {
                            flag_nondup = false;
                            break;
                        }
                    }
                    if (!flag_nondup) break;
                }
            }
            if (flag_nondup)
            {
                cluster_nondup.data[i].data[cnt_nondup] = comp;
                cnt_nondup++;
            }
        }
        cluster_nondup.data[i].data.resize(cnt_nondup);
        
    }

    int cnt = 0;
    for (int i = 0;i < index_cluster;i++)
    {
        cnt += cluster_nondup.data[i].data.size();
    }
    std::cout<< cnt <<std::endl;
}

void PointCloudClass::publishPointCloud(){//データ送信
    pubpc.publish(cloud);
    pub_clus.publish(cluster_nondup);
}

void PointCloudClass::clearMessages(){
    rawcloud.clear();
    cloud.data.clear();
    cloud.index.clear();
    cluster.data.clear();
    cluster_nondup.data.clear();
}