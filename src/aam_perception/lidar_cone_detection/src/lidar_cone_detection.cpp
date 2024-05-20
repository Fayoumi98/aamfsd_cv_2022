
#include <stdio.h>
#include <iostream>
#include <vector>
#include <ros/ros.h>
#include "ros/ros.h"
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>

#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl-1.8/pcl/sample_consensus/ransac.h>
#include <pcl-1.8/pcl/sample_consensus/sac_model_plane.h>
#include <pcl-1.8/pcl/filters/extract_indices.h>
#include <pcl-1.8/pcl/ModelCoefficients.h>
#include <pcl-1.8/pcl/segmentation/sac_segmentation.h>
#include <pcl-1.8/pcl/conversions.h>
#include <pcl-1.8/pcl/search/kdtree.h>
#include <pcl-1.8/pcl/segmentation/extract_clusters.h>




using namespace std;
ros::Publisher no_ground_pc_pub;
ros::Publisher geomtrical_filter_pc_pub;
ros::Publisher clustering_pc_pub;





void callback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL(*cloud_msg,pcl_pc2);
  pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);


  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

  pcl::SACSegmentation<pcl::PointXYZ> seg;

  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);     //Set model type
  seg.setMethodType(pcl::SAC_RANSAC);       //Set random sampling consistency method type
  seg.setMaxIterations(3000);              //Indicates the maximum distance from the point to the estimated model,
  seg.setDistanceThreshold(0.03);           //Set the distance threshold. The distance threshold determines the condition that the point is considered to be an inside point.
  seg.setInputCloud(temp_cloud);
  seg.segment(*inliers, *coefficients);


  if (inliers->indices.size() == 0)
  {
      cout<<"error! Could not found any inliers!"<<endl;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);


  // extract ground
  // Extract the segmented point set on the plane from the point cloud
  pcl::ExtractIndices<pcl::PointXYZ> extractor;//Point extraction object
  extractor.setInputCloud(temp_cloud);
  extractor.setIndices(inliers);
  extractor.setNegative(true);
  extractor.filter(*cloud_filtered);
  // vise-versa, remove the ground not just extract the ground
  // just setNegative to be true
  cout << "filter done."<<endl;


  // PCL pointcloud to pointcloud2
  //sensor_msgs::PointCloud2 output_pc;
  //pcl_conversions::moveFromPCL(cloud_filtered,output_pc)
  
  // Publish the data
  no_ground_pc_pub.publish (cloud_filtered);

  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud(cloud_filtered);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance (0.02); // 2cm
  ec.setMinClusterSize (100);
  ec.setMaxClusterSize (25000);
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud_filtered);
  ec.extract (cluster_indices);


  int j = 0;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
    for (const auto& idx : it->indices)
      cloud_cluster->push_back ((*cloud_filtered)[idx]); //*
    cloud_cluster->header.frame_id="os1_sensor";
    cloud_cluster->width = cloud_cluster->size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

    std::cout << "PointCloud representing the Cluster: " << cloud_cluster->size () << " data points." << std::endl;
    j++;
    clustering_pc_pub.publish(cloud_cluster);
  }

  


  pcl::PointCloud<pcl::PointXYZ> cloud;

  BOOST_FOREACH (const pcl::PointXYZ &pt, cloud_filtered->points)
    
    { 
      pcl::PointXYZ test;

      if (pt.x >= 0.5 && pt.x <= 20 && pt.y >= -10 && pt.y <= 10)
      {
        float tmp[3];
        tmp[0] = pt.x;
        tmp[1] = pt.y;
        tmp[2] = pt.z;
        //printf ("\t(%f, %f, %f)\n", pt.x, pt.y, pt.z);

        test.x = tmp[0];
        test.y = tmp[1];
        test.z = tmp[2];

        cloud.points.push_back(test);

      }
      sensor_msgs::PointCloud2 geometric_filtered_cloud;
      pcl::toROSMsg(cloud,geometric_filtered_cloud);
      geometric_filtered_cloud.header.frame_id="os1_sensor";
      geomtrical_filter_pc_pub.publish(geometric_filtered_cloud);
      
    
    }





}







int main(int argc, char **argv)
{
  ros::init(argc, argv, "lidar_cone_detection");
  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("/ouster_128_points", 1, callback);
  no_ground_pc_pub = nh.advertise<sensor_msgs::PointCloud2> ("/ground_removal_points", 1);
  clustering_pc_pub = nh.advertise<sensor_msgs::PointCloud2> ("/clustering_points", 1);
  geomtrical_filter_pc_pub = nh.advertise<sensor_msgs::PointCloud2> ("/geomtrical_filter_points", 1);


  
  
  ros::spin();
}
