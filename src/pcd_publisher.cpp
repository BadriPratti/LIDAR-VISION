// pcd_publisher.cpp
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

int main(int argc, char** argv) {
    if (argc != 2) {
        ROS_ERROR("Usage: pcd_publisher <path_to_pcd_file>");
        return -1;
    }

    ros::init(argc, argv, "pcd_publisher");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2>("/lidar_points", 1);

    pcl::PointCloud<pcl::PointXYZ> cloud;
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(argv[1], cloud) == -1) {
        ROS_ERROR("Couldn't read file %s", argv[1]);
        return -1;
    }

    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(cloud, output);
    output.header.frame_id = "map";

    ros::Rate loop_rate(1);
    while (ros::ok()) {
        pub.publish(output);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
