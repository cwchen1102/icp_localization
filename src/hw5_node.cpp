#include "ros/ros.h"
#include "std_msgs/String.h"
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/impl/point_types.hpp>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/filters/voxel_grid.h>
#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_broadcaster.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <math.h>
#define PI 3.14159265

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_RGB (new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr Final_RGB (new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZ> Final;
Eigen::Matrix4f trans;

pcl::PointCloud<pcl::PointXYZ>::Ptr downsample(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& msg)
{
    pcl::PCLPointCloud2::Ptr pcl_cloud2 (new pcl::PCLPointCloud2());
    pcl::PCLPointCloud2::Ptr pcl_cloud2_filtered (new pcl::PCLPointCloud2());
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

    pcl::VoxelGrid<pcl::PCLPointCloud2> downsample;
    pcl::toPCLPointCloud2(*msg,*pcl_cloud2);
    downsample.setInputCloud(pcl_cloud2);
    downsample.setLeafSize(0.6f,0.6f,0.6f);
    downsample.filter(*pcl_cloud2_filtered);
    pcl::fromPCLPointCloud2(*pcl_cloud2_filtered,*pcl_cloud_filtered);
    return pcl_cloud_filtered;
}

void sensor_data(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    pcl::fromROSMsg(*msg,*pcl_cloud);
    pcl_cloud_filtered = downsample(pcl_cloud);

    //icp
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(pcl_cloud_filtered);
    icp.setInputTarget(cloud_filtered);
    icp.align(Final,trans);
    std::cout << "has converged:" << icp.hasConverged() << " score: " <<
    icp.getFitnessScore() << std::endl;
    std::cout << icp.getFinalTransformation() << std::endl;
    trans = icp.getFinalTransformation();
}

void tf_brocast(Eigen::Matrix4f trans){
  static tf::TransformBroadcaster br;
  tf::Transform tf_map_scan;
  Eigen::Quaternionf q(trans.topLeftCorner<3, 3>());
  Eigen::Vector3f v = trans.topRightCorner<3, 1>();
  tf_map_scan.setRotation(tf::Quaternion(q.x(), q.y(), q.z(), q.w()));
  tf_map_scan.setOrigin(tf::Vector3(v(0), v(1), v(2)));
  br.sendTransform(tf::StampedTransform(tf_map_scan, ros::Time::now(), "map", "scan"));
}


int main(int argc, char** argv)
{

    //load map
    if (pcl::io::loadPCDFile<pcl::PointXYZ> ("map.pcd", *cloud) == -1) //* load the file
    {
      PCL_ERROR ("Couldn't read file map.pcd \n");
      return (-1);
    }
    std::cout << "cloud Loaded "<< std::endl;
    cloud_filtered = downsample(cloud);

    ros::init(argc, argv, "hw5");
    ros::NodeHandle nh;
    ros::Subscriber sensor_sub = nh.subscribe<sensor_msgs::PointCloud2>("/points_raw", 1000, sensor_data);
    ros::Publisher map_pub = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("/map", 1000);
    ros::Publisher scan_pub = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("/scan", 1000);
    ros::Rate rate(30);

    //initialization
    float degree = -135.0 ;
    Eigen::Matrix4f intial_guess;
    intial_guess << cos ( degree * PI / 180.0 ),sin ( degree * PI / 180.0 ),0.0,0.0,
                    -sin ( degree * PI / 180.0 ),cos ( degree * PI / 180.0 ),0.0,0.0,
                    0.0,0.0,1.0,0.0,
                    0.0,0.0,0.0,1.0;

    trans = intial_guess;

    while (ros::ok())
    {
        cloud_RGB->header.frame_id = "/map";
        cloud_RGB->points.resize (cloud_filtered->width * cloud_filtered->height);
        for (size_t i = 0; i < cloud_filtered->points.size (); ++i)
          {
            cloud_RGB->points[i].x = cloud_filtered->points[i].x;
            cloud_RGB->points[i].y = cloud_filtered->points[i].y;
            cloud_RGB->points[i].z = cloud_filtered->points[i].z;
            cloud_RGB->points[i].g = 255;
        }

        Final_RGB->header.frame_id = "/map";
        Final_RGB->points.resize (Final.width * Final.height);
        for (size_t i = 0; i < Final.points.size (); ++i)
          {
            Final_RGB->points[i].x = Final.points[i].x;
            Final_RGB->points[i].y = Final.points[i].y;
            Final_RGB->points[i].z = Final.points[i].z;
            Final_RGB->points[i].r = 255;
        }

        map_pub.publish(cloud_RGB);
        scan_pub.publish(Final_RGB);
        tf_brocast(trans);
        ros::spinOnce();
        rate.sleep();
    }
    return 0;

}

