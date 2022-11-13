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
#include <pcl/filters/extract_indices.h>    // extract_indices 추가 (바닥 제거)
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

// ==================================RANSAC 바닥 제거============================================================
ros::Publisher pub;
void cloud_cb(const sensor_msgs::PointCloud2& cloud_msg)
{
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromROSMsg(cloud_msg,cloud);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_Ptr(new pcl::PointCloud<pcl::PointXYZ>());
  *cloud_Ptr=cloud;           // pcl::PointCloud 와 pcl::PointCloud ::Ptr 연결

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
  
  seg.setInputCloud(cloud_Ptr);
  seg.segment(*inliers,*coefficients);

  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setInputCloud(cloud_Ptr);
  extract.setIndices(inliers);
  extract.setNegative(true);           //바닥제거하기 위해 inlier를 없앤다
  extract.filter(*inlierPoints_neg);
  // // ROS_INFO_STREAM("Model coefficients : "<<coefficients->values[0]
  //                   <<" "<<coefficients->values[1]<<" "<<coefficients->values[2]<<" "<<coefficients->values[3]
  //                   <<std::endl);
  
  // pcl::copyPointCloud<pcl::PointXYZ>(*cloud_Ptr,*inliers,*inlierPoints_neg);
  

  // 결과값을 포인터로 받음
  // 포인터를 PCLPointCloud2 로 변경 그리고 다시 sensor_msgs 로 변경 후 publish
  pcl::PCLPointCloud2 outlier_cloud;
  sensor_msgs::PointCloud2 outlier_cloud_msg;
  pcl::toPCLPointCloud2(*inlierPoints_neg,outlier_cloud);
  pcl_conversions::fromPCL(outlier_cloud,outlier_cloud_msg);


  pub.publish(outlier_cloud_msg);
  
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
  pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);
  // Spin
  ros::spin ();
}
