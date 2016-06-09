#ifndef FAST_OBJECT_SEGMENTATION
#define FAST_OBJECT_SEGMENTATION

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

namespace cus_obj_seg
{
    typedef pcl::PointXYZ PointType;
    typedef pcl::PointCloud<pcl::PointXYZ> Cloud3D;

    class FastObjectSegmentation
    {

    private:
        float leaf_size_;
        int neighbors_to_analyze_;
        double std_deviation_;
        double min_angle_;
        double max_angle_;
        int ransac_iterations_;
        double threshold_distance_;
        double min_object_height_;
        double max_object_height_;
        double cluster_tolerance_;
        int min_cluster_size_;
        int max_cluster_size_;
        float maximum_distance_thresh_;
        std::vector<pcl::PointIndices> cluster_indices_;
        Cloud3D::Ptr input_cloud_;
        // Cloud3D::Ptr pure_cloud_;
        Cloud3D::Ptr objects_cloud_;

        void passthroughFilter(float max_point);
        void voxelFilter();
        void removeNoise();
        int getPrism();
        void clusterExtraction();
        bool seperateClusters(
                std::vector<pcl::PointIndices>::const_iterator obj_clusters_index,
                Cloud3D::Ptr& cloud_cluster
                );

    public:
        FastObjectSegmentation()
        {
            leaf_size_ = 0.02;
            neighbors_to_analyze_ = 20;
            std_deviation_ = 1.0;
            min_angle_ = 0;
            max_angle_ = 1.57;
            ransac_iterations_ = 1000;
            threshold_distance_ = 0.01;
            min_object_height_ = 0.02;
            max_object_height_ = 0.5;
            cluster_tolerance_ = 0.02;
            min_cluster_size_ = 20;
            max_cluster_size_ = 1000;
            maximum_distance_thresh_ = 1.5f;
        }

        void setLeafSize(float leaf_size)
        {
            leaf_size_ = leaf_size;
        }

        void setNeighbors_to_analyze(int neighbors_to_analyze)
        {
            neighbors_to_analyze_ = neighbors_to_analyze;
        }

        void setStdDeviation(double std_deviation)
        {
            std_deviation_ = std_deviation;
        }

        void setMinAngle(double min_angle)
        {
            min_angle_ = min_angle;
        }

        void setMaxAngle(double max_angle)
        {
            max_angle_ = max_angle;
        }

        void setRansacIterations(int ransac_iterations)
        {
            ransac_iterations_ = ransac_iterations;
        }

        void setThresholdDistance(double threshold_distance)
        {
            threshold_distance_ = threshold_distance;
        }

        void setMinObjectHeight(double min_object_distance)
        {
            min_object_height_ = min_object_distance;
        }

        void setMaxObjectHeight(double max_object_distance)
        {
            max_object_height_ = max_object_distance;
        }

        void setClusterTolerance(double cluster_tolerance)
        {
            cluster_tolerance_ = cluster_tolerance;
        }

        void setMinClusterSize(int min_cluster_size)
        {
            min_cluster_size_ = min_cluster_size;
        }

        void setMaxClusterSize(int max_cluster_size)
        {
            max_cluster_size_ = max_cluster_size;
        }

        void setInputCloud(const Cloud3D::Ptr &input_cloud)
        {
            input_cloud_ = input_cloud;
            //pure_cloud_ = input_cloud;
        }

        void getClusterIndices(std::vector<pcl::PointIndices> &cluster_indices)
        {
            cluster_indices = cluster_indices_;
        }

        bool compute(std::vector<Cloud3D::Ptr>& clusters);

        void setMaximum_distance_thresh(float value)
        {
            maximum_distance_thresh_ = value;
        }
    };


}

#endif
