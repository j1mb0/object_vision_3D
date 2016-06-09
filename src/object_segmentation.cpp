#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/common.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/console/time.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <dynamic_reconfigure/server.h>
#include "pcl_ros/point_cloud.h"
#include "geometries.hpp"
#include "ros/ros.h"
#include "fast_segmentation.hpp"
#include "object_vision/segmentationConfig.h"

typedef pcl::PointXYZ PointType;
typedef pcl::PointCloud<pcl::PointXYZ> CloudType;

ros::Publisher obj_seg_marker_pub;

static object_vision::segmentationConfig last_config;
static cus_obj_seg::FastObjectSegmentation cus_dps;
static size_t prev_marker_size;

// Function Prototypes
void obj_seg_callback(const sensor_msgs::PointCloud2ConstPtr& msg_cloud);
void cfg_cb(object_vision::segmentationConfig &config, uint32_t level);
void clearOldMarkers(std::string frame_id, size_t num_markers, std::string ns1, std::string ns2);



int main(int argc, char **argv)
{
    ros::init(argc, argv, "quanta_obj_seg");

    ros::NodeHandle nh;
    std::string sub_topic = nh.resolveName("camera/depth/points");
    std::string pub_topic = nh.resolveName("/segmented_obj_markers");
    uint32_t queue_size = 1;

 //   ros::ServiceServer service = n.advertiseService("com_of_object",move);

    // to create a subscriber, you can do this (as above):
    ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> (sub_topic, queue_size, obj_seg_callback);
    obj_seg_marker_pub = nh.advertise<visualization_msgs::MarkerArray> (pub_topic, 1);

    dynamic_reconfigure::Server<object_vision::segmentationConfig> server;
    dynamic_reconfigure::Server<object_vision::segmentationConfig>::CallbackType f;

    f = boost::bind(&cfg_cb, _1, _2);
    server.setCallback(f);

    ros::spin();

    return 0;

}

void obj_seg_callback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{

    // initialize variables
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_original (new pcl::PointCloud<pcl::PointXYZRGB>);

    visualization_msgs::MarkerArray marker_array;

    pcl::fromROSMsg (*cloud_msg, *cloud_original);

    std::string marker_ns_obj = "object";
    std::string marker_ns_text = "object_desc";
    std::string frame_id = "camera_depth_optical_frame";


    // enable variables for time logging
    boost::posix_time::ptime time_before_execution;
    boost::posix_time::ptime time_after_execution;
    boost::posix_time::time_duration difference;

    std::vector<CloudType::Ptr> clusters;
    std::vector<pcl::PointIndices> indices;

    pcl::ScopeTime obj_seg_process ("Object Segmentation Callback frame processing ------------- ");

    CloudType::Ptr xyz_points (new CloudType);
    pcl::copyPointCloud (*cloud_original, *xyz_points);


    pcl::console::TicToc tt_seg;
    tt_seg.tic();

    // Segment Objects from Scene
    cus_dps.setInputCloud(xyz_points);
    cus_dps.compute(clusters);
    cus_dps.getClusterIndices(indices);

    std::cout << "[Segmentation done, " << tt_seg.toc() << " ms] \n";

    std::stringstream indicies_info;
    indicies_info << "Indicies size " << indices.size() << " Clusters size " << clusters.size() << std::endl;
    //ROS_INFO(indicies_info.str());

    for (size_t i = 0; i < clusters.size (); i++)
    {

        std::ostringstream desc_text;
        desc_text << "Object #" << i;
        // compute bounding box marker
        c44::BoundingBox obj_box(clusters[i], frame_id, desc_text.str(), i, marker_ns_obj, marker_ns_text);
        // push bounding box marker into array
        marker_array.markers.push_back(obj_box.obj_marker);
        marker_array.markers.push_back(obj_box.text_marker);

    }

    if(marker_array.markers.size() > 0)
    {
        clearOldMarkers(frame_id, prev_marker_size, marker_ns_obj, marker_ns_text);
        obj_seg_marker_pub.publish(marker_array);

        prev_marker_size = marker_array.markers.size();
    }
}


void cfg_cb(object_vision::segmentationConfig &config, uint32_t level) {
    ROS_INFO("Reconfigure Request...");
    //  ROS_INFO("Reconfigure Request: %d %f %s %s %d",
//            config.int_param, config.double_param,
//            config.str_param.c_str(),
//            config.bool_param?"True":"False",
//            config.size);

    cus_dps.setLeafSize(config.downsample_size);
    ROS_INFO("Downsample leaf size: %f", config.downsample_size);

    cus_dps.setMinAngle(config.min_angle);
    ROS_INFO("Min Radius Angle: %f", config.min_angle);

    cus_dps.setMaxAngle(config.max_angle);
    ROS_INFO("Max Radius Angle: %f", config.max_angle);

    cus_dps.setClusterTolerance(config.cluster_tolerance);
    ROS_INFO("Cluster Distance Tolerance: %f", config.cluster_tolerance);

    cus_dps.setMinClusterSize(config.min_cluster_size);
    ROS_INFO("Minimum Cluster Size: %d", config.min_cluster_size);

    cus_dps.setMinObjectHeight(config.object_min_height);
    ROS_INFO("Min Object Height from Plane: %f", config.object_min_height);

    cus_dps.setMaxObjectHeight(config.object_max_height);
    ROS_INFO("Max Object Height from Plane: %f", config.object_max_height);

    cus_dps.setMaximum_distance_thresh(config.maximum_distance_thresh);
    ROS_INFO("Cuttoff Distance from Camera Origin: %f", config.maximum_distance_thresh);

    last_config = config;
}

void clearOldMarkers(std::string frame_id, size_t num_markers, std::string ns1, std::string ns2)
{

pcl::ScopeTime cluster_process_time ("Deleting old markers ------------------ ");
  visualization_msgs::MarkerArray del_marker_array;
  for (int id=0; id < num_markers; id++)
    {
      visualization_msgs::Marker delete_obj_marker;
      delete_obj_marker.header.stamp = ros::Time::now();
      delete_obj_marker.header.frame_id = frame_id;
      delete_obj_marker.id = id;
      delete_obj_marker.action = visualization_msgs::Marker::DELETE;
      delete_obj_marker.ns = ns1;
      del_marker_array.markers.push_back(delete_obj_marker);

      visualization_msgs::Marker delete_text_marker;
      delete_text_marker.header.stamp = ros::Time::now();
      delete_text_marker.header.frame_id = frame_id;
      delete_text_marker.id = id;
      delete_text_marker.action = visualization_msgs::Marker::DELETE;
      delete_text_marker.ns = ns2;
      del_marker_array.markers.push_back(delete_text_marker);

    }
  obj_seg_marker_pub.publish(del_marker_array);
}


