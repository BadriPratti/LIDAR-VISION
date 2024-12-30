#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/search/kdtree.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/pcl_visualizer.h>

typedef pcl::PointXYZ point_t;
typedef pcl::PointCloud<pcl::PointXYZ> cloud_t;

void preprocessPointCloud(cloud_t::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr normals) {
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setInputCloud(cloud);
    vg.setLeafSize(0.02f, 0.02f, 0.02f);
    vg.filter(*cloud);

    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    ne.setSearchMethod(tree);
    ne.setInputCloud(cloud);
    ne.setKSearch(50);
    ne.compute(*normals);
}

void detectEdgesWithGradient(cloud_t::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr normals, cloud_t::Ptr edges, float threshold) {
    for (size_t i = 0; i < cloud->points.size(); ++i) {
        float gradient = 0.0f;
        for (size_t j = 0; j < normals->points.size(); ++j) {
            float diff = std::pow(normals->points[i].normal_x - normals->points[j].normal_x, 2) +
                         std::pow(normals->points[i].normal_y - normals->points[j].normal_y, 2) +
                         std::pow(normals->points[i].normal_z - normals->points[j].normal_z, 2);
            gradient += std::sqrt(diff);
        }
        if (gradient > threshold) {
            edges->points.push_back(cloud->points[i]);
        }
    }
    ROS_INFO("Detected %lu edge points.", edges->points.size());  // Added logging
}

void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {
    cloud_t::Ptr cloud(new cloud_t);
    pcl::fromROSMsg(*cloud_msg, *cloud);

    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    preprocessPointCloud(cloud, normals);

    cloud_t::Ptr edges(new cloud_t);
    float threshold = 0.1; // Adjust based on your dataset
    detectEdgesWithGradient(cloud, normals, edges, threshold);

    ROS_INFO("Original cloud size: %lu points", cloud->points.size());  // Added logging
    ROS_INFO("Edges cloud size: %lu points", edges->points.size());    // Added logging

    // Visualize the result
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Edge Detection"));
    viewer->setBackgroundColor(0, 0, 0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_color_handler(cloud, 255, 255, 255);
    viewer->addPointCloud(cloud, cloud_color_handler, "original cloud");
    
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> edges_color_handler(edges, 255, 0, 0);
    viewer->addPointCloud(edges, edges_color_handler, "edges");

    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "edges"); // Make edges larger
    viewer->spin();
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "edge_detection_node");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/lidar_points", 1, pointCloudCallback);

    ros::spin();
    return 0;
}

