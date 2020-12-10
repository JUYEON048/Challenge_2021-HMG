#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <pcl/io/pcd_io.h>    
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <sensor_msgs/point_cloud_conversion.h>



ros::Publisher pub;

void points_callback(const sensor_msgs::PointCloud msg){
    
/*
    pcl::PointCloud<pcl::PointXY>::Ptr cloud(new pcl::PointCloud<pcl::PointXY>);
//	pcl::PCLPointCloud cloud;
	pcl::PCLPointCloud2 cloud2;

    pcl::fromROSMsg(msg,cloud); 
	pcl::fromPCLPointCLoud2(*cloud,cloud2);   
	sensor_msgs::PointCloud2 out;
	pcl::toROSMsg(cloud2, out);*/
  
	sensor_msgs::PointCloud2 point2;
	sensor_msgs::convertPointCloudToPointCloud2(msg, point2); 
	pub.publish(point2);    
}

int main(int argc, char** argv)
{
    ros::init(argc,argv,"ugv_plane_node");
    ros::NodeHandle nh;
    ros::NodeHandle p_nh("~");

    ros::Subscriber sub;
    sub = nh.subscribe<sensor_msgs::PointCloud>("/pointcloud/vlp_carto",1,points_callback);
    pub = nh.advertise<sensor_msgs::PointCloud2>("plane_points",1);
    
    ros::spin();

    return 0;
}

