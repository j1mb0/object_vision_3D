/*
 * Fast but low resolution object segmentation class.
 * For Portland State University 2016 Capstone Group #44
 *
 * Sean Hendrickson
 *
 *
 * Class Implemenation
 *
 *
 */
#include "fast_segmentation.hpp"


namespace cus_obj_seg
{

    ///////////////////////////////////////////////////////////////////////////////
    /**
     * desc:  This function cuts off all points outside a given range
     *        on the field passed the the field parameter.
     * param: (in) source_cloud - ptr to point cloud
     *        (out) filtered_cloud - ptr to cloud after filtering
     *        (in) axis - axis to filter values passed as 'x', 'y', or 'z'
     *        (in) min_point - kept points are >= minPoint
     *        (in) max_point - kept points are <= maxPoint
     */
void FastObjectSegmentation::passthroughFilter(float max_point)
{
    pcl::PassThrough<PointType> pass;
    pass.setInputCloud(input_cloud_);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0.1f, max_point);
    pass.filter(*input_cloud_);
}

    ///////////////////////////////////////////////////////////////////////////////
    /**
     * desc: This function uses a voxel filter to downsample a PCD file.
     * param: (in) source_cloud - ptr source_cloudto cloud to be filtered
     *        (out) filtered_cloud - ptr to cloud after filtering
     *        (in) leaf_size - size of voxel in cm^3. (ex. 0.01 = 1 cm)
     */
    void FastObjectSegmentation::voxelFilter()
    {
        pcl::VoxelGrid<PointType> sor;
        sor.setInputCloud(input_cloud_);
        sor.setLeafSize(leaf_size_, leaf_size_, leaf_size_);
        sor.filter(*input_cloud_);
    }

    ///////////////////////////////////////////////////////////////////////////////
    /**
     * desc: This function computes the average distance between each point in a PCD
     *       file and then uses the standard deviation to designate points as outliers.
     *       All outliers are then removed from the resultant point cloud.
     * param: (in) source_cloud - ptr to input cloud
     *        (out) filtered_cloud - ptr to cloud with outliers removed
     *        (in) neighbors_to_analyze - number of nearest neighbors to analyze (ex. 50)
     *        (in) std_deviation - standard deviation used to find outliers (ex. 1.0)
     */
    void FastObjectSegmentation::removeNoise()
    {
        pcl::StatisticalOutlierRemoval<PointType> sor;
        sor.setInputCloud(input_cloud_);
        sor.setMeanK(neighbors_to_analyze_);
        sor.setStddevMulThresh(std_deviation_);
        sor.filter(*input_cloud_);
    }

    ///////////////////////////////////////////////////////////////////////////////
    /**
     * desc: This function takes a source PCD image an segments out objects found
     *       on a plane.
     * param: (in) source_cloud - ptr to input cloud
     *        (out) objects_cloud - returns cloud with objects above plane
     *        (in) min_angle - minimum radius between plane and camera in radians (ex. 0)
     *        (in) max_angle - maximum radius between plane and camera in radians (ex. 1.57)
     *        (in) ransac_iterations - maximum amount of times RANSAC can estimate plane (ex. 1000)
     *        (in) threshold_distance - range of points considered inliers from plane model (ex 0.01 = 1cm)
     *        (in) min_object_distance - lowest point of objects above a plane (ex 0.01 = 1cm)
     *        (in) max_object_distance - highest point of objects above a plane (ex 0.2 = 20cm)
     * pre-cond: all output variables must be declared before function is called.
     * ret: -1 if a plane could not be found else
     *      -2 if source PCD is empty
     *       0 if succesfull
     */
    int FastObjectSegmentation::getPrism()
    {
        pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr plane_indices (new pcl::PointIndices);
        Cloud3D::Ptr inliers_cloud (new Cloud3D);
        objects_cloud_.reset(new Cloud3D);

        // find plane parameters
        pcl::SACSegmentation<PointType> seg;
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setMaxIterations(ransac_iterations_);
        seg.setDistanceThreshold(threshold_distance_);
        seg.setRadiusLimits(min_angle_, max_angle_);  // TODO: may need to use different function
        seg.setInputCloud(input_cloud_);
        seg.segment(*plane_indices, *coefficients);

        std::cerr << "Plane Size: " << plane_indices->indices.size() << std::endl;

        // check if a plane was found
        if (plane_indices->indices.size() == 0)
        {	// No plane was found
            return -1;
        }
        else
        {
            // Create the filtering object
            pcl::ExtractIndices<PointType> extract;

            // Extract the inliers
            extract.setInputCloud(input_cloud_);
            extract.setIndices(plane_indices);
            extract.filter(*inliers_cloud);

            // retrieve the convex hull
            Cloud3D::Ptr convexHull (new Cloud3D);
            pcl::ConvexHull<PointType> chull;
            chull.setInputCloud (inliers_cloud);
            chull.setDimension(2);
            chull.reconstruct(*convexHull);
            // redundant check
            if (chull.getDimension() == 2)
            {
                // segment those points that are in the polygonal prism
                pcl::ExtractPolygonalPrismData<PointType> prism;
                prism.setInputCloud(input_cloud_);
                prism.setInputPlanarHull(convexHull);
                prism.setHeightLimits(min_object_height_, max_object_height_);
                pcl::PointIndices::Ptr objectIndices (new pcl::PointIndices);
                prism.segment (*objectIndices);

                // get all points retrieved by the hull
                extract.setIndices(objectIndices);
                extract.filter(*objects_cloud_);



                // check if any objects were found
                if(0 == objects_cloud_->points.size())
                {	// no objects found
                    return -1;
                }
            }
            else
            {	// the chosen hull is not planar
                return -1;
            }
        }
        return 0;

    }


    ///////////////////////////////////////////////////////////////////////////////
    /**
     * desc: This function uses a "Euclidean Cluster Extraction" algorithm to pair
     *       separate data in a Point Cloud into separate objects.
     * param: (in) source_cloud - ptr to input cloud
     *        (out) cluster_indicies - ojects that holds each cluster as an element of
     *                                 a vector. cluster_indices[0] is cluster 1, and
     *                                 cluster_indices[1] is cluster 2, and so on.
     *        (in) cluster_tolerance - The maximum distance between points in the
     *                                same cluster (e.g. 0.02 = 2cm)
     *        (in) min_cluster_size - minimum number of points in a single cluster
     *        (in) max_cluster_size - minimum number of points in a single cluster
     * pre-cond: cluster_indicies needs to be declared before function call
     */
    void FastObjectSegmentation::clusterExtraction()
    {
        cluster_indices_.clear();
        // create the kdTree object for the search method of extraction
        pcl::search::KdTree<PointType>::Ptr tree (new pcl::search::KdTree<PointType>);
        tree->setInputCloud (objects_cloud_);
        //pcl::PCDWriter writer;

        // store each cluster as a vector of indices
        pcl::EuclideanClusterExtraction<PointType> ec;
        ec.setClusterTolerance (cluster_tolerance_);
        ec.setMinClusterSize (min_cluster_size_);
        ec.setMaxClusterSize (max_cluster_size_);
        ec.setSearchMethod (tree);
        ec.setInputCloud (objects_cloud_);
        ec.extract (cluster_indices_);
    }

    ///////////////////////////////////////////////////////////////////////////////
    /**
     * desc:
     * Seperates a cluster discovered by Euclidean Extraction from the
     * cloud of clusters. Not sure why we are not using indicies extraction for this.
     * #TODO - see if manual indicies extraction was actually neccessary or some
     *          weird tutorial related thing
     *
     * To separate each cluster out of the vector<PointIndices> we have to iterate
     * through cluster_indices, create a new PointCloud for each entry
     * and write all points of the current cluster in the PointCloud.
     * param: (inout) it - Iterator holding reference to current cluster to seperate
     *        (in) cloud_of_objs - point cloud of se
     *        (out) cloud_cluster - point cloud of seperated object
     */
    bool FastObjectSegmentation::seperateClusters(
            std::vector<pcl::PointIndices>::const_iterator obj_clusters_index,
            Cloud3D::Ptr& cloud_cluster
            )
    {
        bool last_cluster = false;

        if(obj_clusters_index != cluster_indices_.end ())
        {
            //cloud_cluster = new Cloud3D::Ptr();
            for (std::vector<int>::const_iterator pit = obj_clusters_index->indices.begin (); pit != obj_clusters_index->indices.end (); ++pit)
            {
                cloud_cluster->points.push_back (objects_cloud_->points[*pit]); //*
            }

            // manually set the parameters
            cloud_cluster->width = cloud_cluster->points.size ();
            cloud_cluster->height = 1;
            cloud_cluster->is_dense = true;

            // Set ROS specific parameters
            //cloud_cluster->header.frame_id = "/camera_depth_frame";
            //cloud_cluster->header.stamp = ros::Time::now().toSec();

        }
        else
        {
            // all clusters processed
            last_cluster = true;
        }

        return last_cluster;
    }


    bool FastObjectSegmentation::compute(std::vector<Cloud3D::Ptr>& clusters)
    {
        bool success = false;
        int result;
        cluster_indices_.clear();
        objects_cloud_.reset(new Cloud3D);

        // check for valid image
        if(input_cloud_->size() < 0)
        {
            return success;
        }

        voxelFilter();  // call voxel filter

        passthroughFilter(maximum_distance_thresh_);

        // seperate prism on top of plane into a new cloud (that hopefully contains the objects)
        result = getPrism();
        std::cout << "size of objects cloud: width " << objects_cloud_->width << ", height " << objects_cloud_->height << std::endl;


        // check for errors from getPrism()
        if(result < 0)
        {
            std::cout << "No plane found in getPrism" << std::endl;
            //pub.publish(cloud_original);
        }
        else
        {
            // remove noise from objects PCD using gaussian filter
            removeNoise();

            clusterExtraction();
            std::cout << "Found " << cluster_indices_.size() << " clusters." << std::endl;

            if(cluster_indices_.size() > 0)
            {
               //clusters.clear();

                // Look through each cluster for an object and a hand
                int j = 0;
                std::vector<pcl::PointIndices>::const_iterator obj_clusters_index = cluster_indices_.begin ();
                while(obj_clusters_index != cluster_indices_.end ())
                {
                    //clusters[j] = Cloud3D::Ptr (new Cloud3D);
                    Cloud3D::Ptr cluster_cloud (new Cloud3D);

                    seperateClusters(obj_clusters_index, cluster_cloud);//clusters[j]);
                    clusters.push_back(cluster_cloud);
                    ++obj_clusters_index;     // vectors are non-trivial class members, so pre-increment has better performance than it++
                    j++;
                }
                //clusters.resize(j);
                success = true;
                std::cout << "Cluster size: " << clusters.size() << std::endl;
            }
        }

        return success;

    }
}
