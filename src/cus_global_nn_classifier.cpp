/*
 * global_nn_classifier.cpp
 *
 *  Created on: Mar 9, 2012
 *      Author: aitor
 */

#include "cus_global_nn_classifier.hpp"
#include "pcl/apps/3d_rec_framework/utils/metrics.h"

//Instantiation
template class obj_seg::rec_3d_framework::GlobalNNPipeline<flann::L1, pcl::PointXYZ, pcl::VFHSignature308>;
template class obj_seg::rec_3d_framework::GlobalNNPipeline<Metrics::HistIntersectionUnionDistance, pcl::PointXYZ, pcl::VFHSignature308>;
template class obj_seg::rec_3d_framework::GlobalNNPipeline<flann::L1, pcl::PointXYZ, pcl::ESFSignature640>;

template class obj_seg::rec_3d_framework::GlobalClassifier<pcl::PointXYZ>;

