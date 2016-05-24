/*
 * test_training.cpp
 *
 *  Created on: Mar 9, 2012
 *      Author: aitor
 */
#include <pcl/pcl_macros.h>
//#include <pcl/apps/3d_rec_framework/pipeline/global_nn_classifier.h>
#include "cus_global_nn_classifier.h"
#include <pcl/apps/3d_rec_framework/pc_source/mesh_source.h>
#include <pcl/apps/3d_rec_framework/feature_wrapper/global/vfh_estimator.h>
#include <pcl/apps/3d_rec_framework/feature_wrapper/global/esf_estimator.h>
#include <pcl/apps/3d_rec_framework/feature_wrapper/global/cvfh_estimator.h>
//#include <pcl/apps/3d_rec_framework/tools/openni_frame_source.h>
#include <pcl/apps/3d_rec_framework/utils/metrics.h>
//#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/apps/dominant_plane_segmentation.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>
#include "pcl_ros/point_cloud.h"
#include "geometries.hpp"
#include "ros/ros.h"
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pwd.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>

#define VISUALIZE_BB

// testing

static obj_seg::rec_3d_framework::GlobalNNPipeline<Metrics::HistIntersectionUnionDistance, pcl::PointXYZ, pcl::VFHSignature308> global;
ros::Publisher pub;
static size_t prev_marker_size;

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
  pub.publish(del_marker_array);
}

void
segmentAndClassify_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    // Set our initial shape type to be a cube
    uint32_t shape = visualization_msgs::Marker::CUBE;

    visualization_msgs::MarkerArray marker_array;
    std::string marker_ns_obj = "object";
    std::string marker_ns_text = "object_desc";
    std::string frame_id = "camera_rgb_optical_frame";

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_scene (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg (*cloud_msg, *cloud_scene);
    cloud_scene->is_dense = false;
//std::cout << "Recieved Point Cloud, width: " << cloud_scene->width << " height: " << cloud_scene->height << " IsDense: " << cloud_scene->is_dense << std::endl;

    float Z_DIST_ = 1.5f;
    float text_scale = 0.02f;

    pcl::ScopeTime frame_process ("Global frame processing ------------- ");
    //frame = camera.snap ();
    pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_points (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud (*cloud_scene, *xyz_points);

//std::cout << "Removing color from scene, width: " << xyz_points->width << "height: " << xyz_points->height << " IsDense: " << cloud_scene->is_dense << std::endl;

    pcl::console::TicToc tt_seg;
    tt_seg.tic();
    //Step 1 -> Segment
    pcl::apps::DominantPlaneSegmentation<pcl::PointXYZ> dps;
    dps.setInputCloud (xyz_points);
    dps.setMaxZBounds (Z_DIST_);
    dps.setObjectMinHeight (0.02);
    dps.setMinClusterSize (50);
    dps.setWSize (9);
    dps.setDistanceBetweenClusters (0.1f);
    dps.setObjectMaxHeight(0.5);

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters;
    std::vector<pcl::PointIndices> indices;
    dps.setDownsamplingSize (0.02f);
    dps.compute_fast (clusters);
    dps.getIndicesClusters (indices);
//std::cout << "Indicies recieved: " << indices.empty() << std::endl;
    Eigen::Vector4f table_plane_;
    dps.getTableCoefficients (table_plane_);
    Eigen::Vector3f normal_plane_ = Eigen::Vector3f (table_plane_[0], table_plane_[1], table_plane_[2]);
    std::cout << "[Segmentation done, " << tt_seg.toc() << " ms] \n";

    pcl::console::TicToc tt;
    tt.tic();

    size_t cur_marker_size = 0;

    for (size_t i = 0; i < clusters.size (); i++)
    {
        std::ostringstream prob_str;
        prob_str.precision (1);

        std::stringstream clus_proc_time_str;
        clus_proc_time_str << "Catagorized cluster " << i << " --------";
        pcl::ScopeTime cluster_process_time (clus_proc_time_str.str().c_str());
//std::cout << "working on " << i << "th cluster" << std::endl;
        std::stringstream cluster_name;
        cluster_name << "cluster_" << i;
        global.setInputCloud (xyz_points);
        global.setIndices (indices[i].indices);
        global.classify ();


        std::vector < std::string > categories;
        std::vector<float> conf;

//std::cout << "Done... getting categories" << std::endl;
      global.getCategory (categories);

//std::cout << "Done... getting confidence" << std::endl;
      global.getConfidence (conf);

      //std::string category = categories[0];
      //Eigen::Vector4f centroid;
      //pcl::compute3DCentroid (*xyz_points, indices[i].indices, centroid);

//std::cout << "Done... found " << categories.size() << " categories" <<  std::endl;
      for (size_t kk = 0; kk < categories.size (); kk++)
      {
        prob_str << "cluster[" << i << "]: " << categories[kk] << " [" << conf[kk] << "]" << std::endl;
        //previous_categories_size++;
      }

#ifdef VISUALIZE_BB
      c44::BoundingBox obj_box(clusters[i], frame_id, prob_str.str(), i, marker_ns_obj, marker_ns_text);
      //std::cout << "Inside loop, Marker " << i << " type: " << obj_box.text_marker.type << std::endl;
      marker_array.markers.push_back(obj_box.obj_marker);
      marker_array.markers.push_back(obj_box.text_marker);
      cur_marker_size += 2;
#endif
    }
#ifdef VISUALIZE_BB
std::cout << "Previous Marker size: " << prev_marker_size << ", Current Marker size: " << cur_marker_size << std::endl;

    clearOldMarkers(frame_id, prev_marker_size, marker_ns_obj, marker_ns_text);
std::cout << "Number of markers to publish" << marker_array.markers.size() << endl;
    pub.publish(marker_array);

    prev_marker_size = marker_array.markers.size();
#endif
    std::cout << "[Classification done, " << tt.toc() << " ms] \n";
std::cout << "Objects segmented: " << clusters.size() << std::endl;
}

int
main (int argc, char ** argv)
{
    prev_marker_size = 0;
    const char* home = getenv("HOME");

    if(!home){
        std::cerr << "Can't find home directory";
    }
    std::string home_path(home);


    //std::string scene_path = "scene.pcd";
    std::string path = home_path + "/classification/models/";
    std::string desc_name = "cvfh";
    std::string training_dir = home_path + "/classification/trained_models/";
    int NN = 10;

    //pcl::console::parse_argument (argc, argv, "-scene_path", scene_path);
    //pcl::console::parse_argument (argc, argv, "-models_dir", path);
    //pcl::console::parse_argument (argc, argv, "-training_dir", training_dir);
    //pcl::console::parse_argument (argc, argv, "-descriptor_name", desc_name);
    //pcl::console::parse_argument (argc, argv, "-nn", NN);

    //pcl::console::parse_argument (argc, argv, "-z_dist", chop_at_z_);
    //pcl::console::parse_argument (argc, argv, "-tesselation_level", views_level_);

    boost::shared_ptr<pcl::rec_3d_framework::MeshSource<pcl::PointXYZ> > mesh_source (new pcl::rec_3d_framework::MeshSource<pcl::PointXYZ>);
    mesh_source->setPath (path);
    mesh_source->setResolution (150);
    mesh_source->setTesselationLevel (1);
    mesh_source->setViewAngle (57.f);
    mesh_source->setRadiusSphere (1.5f);
    mesh_source->setModelScale (1.f);
    mesh_source->generate (training_dir);

    boost::shared_ptr<pcl::rec_3d_framework::Source<pcl::PointXYZ> > cast_source;
    cast_source = boost::static_pointer_cast<pcl::rec_3d_framework::MeshSource<pcl::PointXYZ> > (mesh_source);

    boost::shared_ptr<pcl::rec_3d_framework::PreProcessorAndNormalEstimator<pcl::PointXYZ, pcl::Normal> > normal_estimator;
    normal_estimator.reset (new pcl::rec_3d_framework::PreProcessorAndNormalEstimator<pcl::PointXYZ, pcl::Normal>);
    normal_estimator->setCMR (true);
    normal_estimator->setDoVoxelGrid (true);
    normal_estimator->setRemoveOutliers (true);
    normal_estimator->setFactorsForCMR (3, 7);

    boost::shared_ptr<pcl::rec_3d_framework::CVFHEstimation<pcl::PointXYZ, pcl::VFHSignature308> > vfh_estimator;
    vfh_estimator.reset (new pcl::rec_3d_framework::CVFHEstimation<pcl::PointXYZ, pcl::VFHSignature308>);
    vfh_estimator->setNormalEstimator (normal_estimator);

    boost::shared_ptr<pcl::rec_3d_framework::GlobalEstimator<pcl::PointXYZ, pcl::VFHSignature308> > cast_estimator;
    cast_estimator = boost::dynamic_pointer_cast<pcl::rec_3d_framework::CVFHEstimation<pcl::PointXYZ, pcl::VFHSignature308> > (vfh_estimator);

    global.setDataSource (cast_source);
    global.setTrainingDir (training_dir);
    global.setDescriptorName (desc_name);
    global.setFeatureEstimator (cast_estimator);
    global.setNN (NN);
    global.initialize (false);

    ros::init(argc, argv, "quanta_obj_seg");

    ros::NodeHandle nh;
    std::string sub_topic = nh.resolveName("camera/depth/points");
    std::string pub_topic = nh.resolveName("obj_marker");
    uint32_t queue_size = 1;

    // to create a subscriber, you can do this (as above):
    ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> (sub_topic, queue_size, segmentAndClassify_cb);

    pub = nh.advertise<visualization_msgs::MarkerArray>("visualization_marker", 1);

    ros::spin();

    return 0;
}
