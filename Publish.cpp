#include <iostream>
#include <cmath>
#include <vector>
#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>

#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>    // pass through 추가
#include <pcl/impl/point_types.hpp>
#include<pcl/sample_consensus/model_types.h>
#include<pcl/sample_consensus/method_types.h>
#include<pcl/segmentation/sac_segmentation.h>   //ransac 사용하려고 추가

#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include <set>
#include <boost/format.hpp>

#define PI 3.14159265359

ros::Publisher pub;
void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input_PC2_cloud)
{
  //pcl2 형태로 받아서 pcl1의 xyz 포인트클라우드로 만들어주자
    pcl::PointCloud<pcl::PointXYZL>::Ptr current_sensor_cloud_ptr(new pcl::PointCloud<pcl::PointXYZL>);
    sensor_msgs::PointCloud input_PC1_cloud;
    sensor_msgs::convertPointCloud2ToPointCloud(*input_PC2_cloud, input_PC1_cloud);

    int input_pt_size = input_PC1_cloud.points.size();

    for(int i=0;i<input_pt_size;i++){

        pcl::PointXYZL p;

        p.x = input_PC1_cloud.points[i].x;
        p.y = input_PC1_cloud.points[i].y;
        p.z = input_PC1_cloud.points[i].z;

        p.label = static_cast<std::uint32_t>(input_PC1_cloud.channels[2].values[i]);

        current_sensor_cloud_ptr->points.push_back(p);

		//std::cout<<p.label<<std::endl;
    }
    


  // pcl::PointCloud2 cloud;
  //pcl::PointCloud<pcl::PointXYZ> cloud;
  //pcl::fromROSMsg(*cloud_msg,cloud);
  // pcl_conversions::toPCL(cloud_msg,cloud);
  //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_Ptr(new pcl::PointCloud<pcl::PointXYZ>());
  //pcl::fromPCLPointCloud2(cloud,*cloud_Ptr); //들어가는 형태가 pcl2가 아니라 pcl1으로 들어가서 no matching에러

  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices());

  pcl::PointCloud<pcl::PointXYZ>::Ptr inlierPoints(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr inlierPoints_neg(new pcl::PointCloud<pcl::PointXYZ>());

  // SACSegmentation 을 위해서 seg 를 만들고 방법과 모델과 기준을 정함
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(0.01);
  
  seg.setInputCloud(current_sensor_cloud_ptr);
  seg.segment(*inliers,*coefficients);



  ROS_INFO_STREAM("Model coefficients : "<<coefficients->values[0]
                    <<" "<<coefficients->values[1]<<" "<<coefficients->values[2]<<" "<<coefficients->values[3]
                    <<std::endl);
  
  pcl::copyPointCloud<pcl::PointXYZ>(*cloud_Ptr,*inliers,*inlierPoints);

  // 결과값을 포인터로 받음
  // 포인터를 PCLPointCloud2 로 변경 그리고 다시 sensor_msgs 로 변경 후 publish
  pcl::PCLPointCloud2 inlier_cloud;
  sensor_msgs::PointCloud2 inlier_cloud_msg;
  pcl::toPCLPointCloud2(*inlierPoints,inlier_cloud);
  pcl_conversions::fromPCL(inlier_cloud,inlier_cloud_msg);


  pub.publish(inlier_cloud_msg);
  
}

int main(int argc,char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "my_pcl_tutorial");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  // ros::Subscriber sub = nh.subscribe ("/velodyne_points", 1, cloud_cb);
  ros::Subscriber sub = nh.subscribe ("/carla/ego_vehicle/semantic_lidar", 1, cloud_cb);
  // Create a ROS publisher for the output point cloud
  // pub = nh.advertise<pcl_msgs::ModelCoefficients> ("output", 1);
  pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);
  // Spin
  ros::spin ();
}



//======================= 직접 값을 불러와서 ROI 설정(각도 설정)=======================
// ros::Publisher pub;

// double ROI_theta(double x, double y)
// {
//   double r;
//   double theta;

//   r=sqrt((x*x)+(y*y));
//   theta=acos(x/r)*180/PI;
//   return theta;
// }

// void cloud_cb(const sensor_msgs::PointCloud2& cloud_msg)
// {
//   pcl::PCLPointCloud2 cloud;
//   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_Ptr(new pcl::PointCloud<pcl::PointXYZ>());

//   sensor_msgs::PointCloud2 filtered_cloud_msg;
//   pcl::PCLPointCloud2 filtered_cloud;
//   pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud_Ptr(new pcl::PointCloud<pcl::PointXYZ>());

//   pcl::PointCloud<pcl::PointXYZ> cloudIn;
//   pcl::fromROSMsg(cloud_msg,cloudIn);
  
//   for(unsigned int j=0;j<cloudIn.points.size();j++)
//   {
//     if(ROI_theta(cloudIn.points[j].y,cloudIn.points[j].x)<45)
//     {
//       cloudIn.points[j].x=0;
//       cloudIn.points[j].y=0;
//       cloudIn.points[j].z=0;
//     }
//     if(ROI_theta(cloudIn.points[j].y,cloudIn.points[j].x)>135)
//     {
//       cloudIn.points[j].x=0;
//       cloudIn.points[j].y=0;
//       cloudIn.points[j].z=0;
//     }
//         if(cloudIn.points[j].x<0)
//     {
//       cloudIn.points[j].x=0;
//       cloudIn.points[j].y=0;
//       cloudIn.points[j].z=0;
//     }    
//   }
//   pcl::toPCLPointCloud2(cloudIn,filtered_cloud);
//   pcl_conversions::fromPCL(filtered_cloud,filtered_cloud_msg);
//   pub.publish(filtered_cloud_msg);
// }

// int main(int argc,char** argv)
// {
//   // Initialize ROS
//   ros::init (argc, argv, "my_pcl_tutorial");
//   ros::NodeHandle nh;

//   // Create a ROS subscriber for the input point cloud
//   // ros::Subscriber sub = nh.subscribe ("/velodyne_points", 1, cloud_cb);
//   ros::Subscriber sub = nh.subscribe ("/carla/ego_vehicle/semantic_lidar", 1, cloud_cb);
//   // Create a ROS publisher for the output point cloud
//   pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);
//   // Spin
//   ros::spin ();
// }
//=============================== x 축 y 축 필터링(pass through 사용)==============
// ros::Publisher pub;


// void cloud_cb(const sensor_msgs::PointCloud2& cloud_msg)
// {
//   pcl::PCLPointCloud2 cloud;
//   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_Ptr(new pcl::PointCloud<pcl::PointXYZ>());

//   pcl_conversions::toPCL(cloud_msg,cloud);
//   pcl::fromPCLPointCloud2(cloud,*cloud_Ptr);

//   // pcl::PCLPointCloud2ConstPtr cloud_Ptr(cloud);       // 이러면 cloud 와 cloudPtr이 연결 바로 됨

//   pcl::PCLPointCloud2 filtered_cloud;
//   pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud_Ptr(new pcl::PointCloud<pcl::PointXYZ>());
//   sensor_msgs::PointCloud2 filtered_cloud_msg;


//   float x_filter_limit=20.0;   //20.0m
// // x축으로 filtering
//   pcl::PassThrough<pcl::PointXYZ> xfilter;
//   xfilter.setInputCloud(cloud_Ptr);            // constPtr은 안되고 그냥 Ptr은 되네...
//   xfilter.setFilterFieldName("x");
//   xfilter.setFilterLimits(0,x_filter_limit);    // 차량 전방 방향으로 20m 까지
//   xfilter.filter(*filtered_cloud_Ptr);

//   float y_filter_limit=4.0;   //4m
//   //y축으로 filtering
//   pcl::PassThrough<pcl::PointXYZ> yfilter;
//   yfilter.setInputCloud(filtered_cloud_Ptr);
//   yfilter.setFilterFieldName("y");
//   yfilter.setFilterLimits(-y_filter_limit,y_filter_limit);  // 차량 좌우 4m 까지 
//   yfilter.filter(*filtered_cloud_Ptr);

//   //ptr 를 pcl로 
//   pcl::toPCLPointCloud2(*filtered_cloud_Ptr,filtered_cloud);

//   //pcl 을 sensor_msgs로
//   pcl_conversions::fromPCL(filtered_cloud,filtered_cloud_msg);
  
//   pub.publish(filtered_cloud_msg);
  
// }

// int main(int argc,char** argv)
// {
//   // Initialize ROS
//   ros::init (argc, argv, "my_pcl_tutorial");
//   ros::NodeHandle nh;

//   // Create a ROS subscriber for the input point cloud
//   // ros::Subscriber sub = nh.subscribe ("/velodyne_points", 1, cloud_cb);
//   ros::Subscriber sub = nh.subscribe ("/carla/ego_vehicle/semantic_lidar", 1, cloud_cb);
//   // Create a ROS publisher for the output point cloud
//   pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);
//   // Spin
//   ros::spin ();
// }

//===============================================try(voxel grid roswiki)==============

// ros::Publisher pub;

// void 
// cloud_cb (const sensor_msgs::PointCloud2& cloud_msg)
// {
//   pcl::PCLPointCloud2* cloud= new pcl::PCLPointCloud2;
//   pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
//   pcl::PCLPointCloud2 cloud_filtered;

//   pcl_conversions::toPCL(cloud_msg,*cloud);

//   pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
//   sor.setInputCloud(cloudPtr);
//   sor.setLeafSize(1,1,1);
//   sor.filter(cloud_filtered);

//   sensor_msgs::PointCloud2 output;
//   pcl_conversions::fromPCL(cloud_filtered,output);


//   pub.publish(output);

// }

// int
// main (int argc, char** argv)
// {
//   // Initialize ROS
//   ros::init (argc, argv, "my_pcl_tutorial");
//   ros::NodeHandle nh;

//   // Create a ROS subscriber for the input point cloud
//   // ros::Subscriber sub = nh.subscribe ("/velodyne_points", 1, cloud_cb);
//   ros::Subscriber sub = nh.subscribe ("/carla/ego_vehicle/semantic_lidar", 1, cloud_cb);
//   // Create a ROS publisher for the output point cloud
//   pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);
//   // Spin
//   ros::spin ();
// }

//=========================================2. voxel down sampling(example)===============
// ros::Publisher pub;

// // This is to save on typing
// typedef pcl::PointCloud<pcl::PointXYZ> point_cloud_t;

// void cloud_cb (const sensor_msgs::PointCloud2& ros_pc)
// {
//     // See http://wiki.ros.org/hydro/Migration for the source of this magic.
//     pcl::PCLPointCloud2 pcl_pc; // temporary PointCloud2 intermediary
//     pcl_conversions::toPCL(ros_pc, pcl_pc);

//     // Convert point cloud to PCL native point cloud
//     point_cloud_t::Ptr input_ptr(new point_cloud_t());
//     pcl::fromPCLPointCloud2(pcl_pc, *input_ptr);

//     // Set up VoxelGrid filter to bin into 10cm grid
//     pcl::VoxelGrid<pcl::PointXYZ> sor;
//     sor.setInputCloud(input_ptr);
//     sor.setLeafSize(0.1, 0.1, 0.1);

//     // Create output point cloud
//     point_cloud_t::Ptr output_ptr(new point_cloud_t());

//     // Run filter
//     sor.filter(*output_ptr);

//     // Now covert output back from PCL native type to ROS
//     sensor_msgs::PointCloud2 ros_output;
//     pcl::toPCLPointCloud2(*output_ptr, pcl_pc);
//     pcl_conversions::fromPCL(pcl_pc, ros_output);

//     // Publish the data
//     pub.publish(ros_output);
// }

// int main (int argc, char** argv)
// {
//     // Initialize ROS
//     ros::init (argc, argv, "pcl_voxel");
//     ros::NodeHandle nh;

//     // Create a ROS subscriber for the input point cloud
//     ros::Subscriber sub = nh.subscribe("/velodyne_points", 1, cloud_cb);

//     // Create a ROS publisher for the output point cloud
//     pub = nh.advertise<sensor_msgs::PointCloud2>("/velodyne_points_new", 1);

//     // Spin
//     ros::spin ();
// }

//==========================1. ros 실습 준비===================
// using namespace std;
// ros::Publisher pub;

// void 
// cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
// {
//   // ... do data processing

//   sensor_msgs::PointCloud2 output;
  
//   // Publish the data
//   pub.publish (input);
// }

// int
// main (int argc, char** argv)
// {
//   // Initialize ROS
//   ros::init (argc, argv, "my_pcl_tutorial");
//   ros::NodeHandle nh;

//   // Create a ROS subscriber for the input point cloud
//   ros::Subscriber sub = nh.subscribe ("/carla/ego_vehicle/semantic_lidar", 1, cloud_cb);

//   // Create a ROS publisher for the output point cloud
//   pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);

//   // Spin
//   ros::spin ();
// }


//==============================승휘님 코드===================
// ros::Publisher pub;

// void Callback(const sensor_msgs::PointCloud2ConstPtr& input_PC2_cloud)
// {
//     pcl::PointCloud<pcl::PointXYZL>::Ptr current_sensor_cloud_ptr(new pcl::PointCloud<pcl::PointXYZL>);
//     sensor_msgs::PointCloud input_PC1_cloud;
//     sensor_msgs::convertPointCloud2ToPointCloud(*input_PC2_cloud, input_PC1_cloud);

//     int input_pt_size = input_PC1_cloud.points.size();

//     for(int i=0;i<input_pt_size;i++){

//         pcl::PointXYZL p;

//         p.x = input_PC1_cloud.points[i].x;
//         p.y = input_PC1_cloud.points[i].y;
//         p.z = input_PC1_cloud.points[i].z;

//         p.label = static_cast<std::uint32_t>(input_PC1_cloud.channels[2].values[i]);

//         current_sensor_cloud_ptr->points.push_back(p);

// 		//std::cout<<p.label<<std::endl;
//     }

//     // pointcloud에서 버스정류장과 보행자만 뽑아냄
//     pcl::PointCloud<pcl::PointXYZL>::Ptr Hwi_cloud_ptr(new pcl::PointCloud<pcl::PointXYZL>);
    
//     for(auto& current_points : current_sensor_cloud_ptr->points){
//         if(current_points.label == 4 || current_points.label == 0 ){ //pedestrian & bus-stop tag
//             Hwi_cloud_ptr->points.push_back(current_points);
// 			//std::cout<<current_points.label<<std::endl;
//         }
//     }    
//     sensor_msgs::PointCloud2 Hwi_roi;
//     pcl::toROSMsg<pcl::PointXYZL>(*Hwi_cloud_ptr, Hwi_roi);
//     Hwi_roi.header = input_PC2_cloud->header;
//     pub.publish(Hwi_roi);
// }

// int main(int argc, char** argv)
// { 
// 	ros::init(argc, argv, "input");
// 	ros::NodeHandle nh;

// 	ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> ("/carla/ego_vehicle/semantic_lidar", 1, Callback);
// 	pub = nh.advertise<sensor_msgs::PointCloud2> ("/HWI/pedestrian_busstop/semantic_lidar", 1);
	
// 	ros::spin();
// }