//
//  Geometries.cpp
//  PCLTestbench
//
//  Creates bounding boxes + ROS markers for a point cloud
//
//  Matthew Whiteside
//
//  Created on 2/10/16.
//

#include "geometries.hpp"
using namespace c44;

BoundingBox::BoundingBox(Cloud3D::Ptr cloud)
{
ScopeTime bounding_box_time ("Created bounding box ------------- ");
  MomentOfInertiaEstimation<PointType> feature_extractor;
  feature_extractor.setInputCloud (cloud);
  feature_extractor.compute();
  feature_extractor.getMomentOfInertia (moment_of_inertia);
  feature_extractor.getEccentricity (eccentricity);
  feature_extractor.getAABB (min_point_AABB, max_point_AABB);
  feature_extractor.getOBB (min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);
  feature_extractor.getEigenValues (major_value, middle_value, minor_value);
  feature_extractor.getEigenVectors (major_vector, middle_vector, minor_vector);
  feature_extractor.getMassCenter (centroid);
  width = max_point_OBB.x - min_point_OBB.x;
  height = max_point_OBB.y - min_point_OBB.y;
  depth = max_point_OBB.z - min_point_OBB.z;
}

BoundingBox::BoundingBox(Cloud3D::Ptr cloud, std::string frame, std::string text, int id, std::string ns1, std::string ns2)
{
ScopeTime bounding_box_time ("Created bounding box ------------- ");
  MomentOfInertiaEstimation<PointType> feature_extractor;
  feature_extractor.setInputCloud (cloud);
  feature_extractor.compute();
  //feature_extractor.compute();
  //feature_extractor.getMomentOfInertia (moment_of_inertia);
  //feature_extractor.getEccentricity (eccentricity);
  //feature_extractor.getAABB (min_point_AABB, max_point_AABB, position_AABB, rotational_matrix_AABB);
  feature_extractor.getOBB (min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);
  //feature_extractor.getEigenValues (major_value, middle_value, minor_value);
  //feature_extractor.getEigenVectors (major_vector, middle_vector, minor_vector);
  //feature_extractor.getMassCenter (centroid);
  width = max_point_OBB.x - min_point_OBB.x;
  height = max_point_OBB.y - min_point_OBB.y;
  depth = max_point_OBB.z - min_point_OBB.z;
//  width = max_point_AABB.x - min_point_AABB.x;
//  height = max_point_AABB.y - min_point_AABB.y;
//  depth = max_point_AABB.z - min_point_AABB.z;

  obj_marker.header.frame_id = frame;
  text_marker.header.frame_id = frame;

  obj_marker.header.stamp = ros::Time::now();
  text_marker.header.stamp = ros::Time::now();

  obj_marker.ns = ns1;
  obj_marker.id = id;

  text_marker.ns = ns2;
  text_marker.id = id;

  obj_marker.type = visualization_msgs::Marker::CUBE;
  text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;

  obj_marker.action = visualization_msgs::Marker::ADD;
  text_marker.action = visualization_msgs::Marker::ADD;

  Eigen::Quaternionf eig_quat(rotational_matrix_OBB);
  tf::quaternionEigenToTF(eig_quat.cast<double>(), quat);

  obj_marker.pose.position.x = position_OBB.x; //centroid[0];
  text_marker.pose.position.x = position_OBB.x + 0.3;//centroid[0];
  obj_marker.pose.position.y = position_OBB.y;//centroid[1];
  text_marker.pose.position.y = position_OBB.y - 0.3;//centroid[1];
  obj_marker.pose.position.z = position_OBB.z;//centroid[2];
  text_marker.pose.position.z = position_OBB.z + 0.3;//centroid[2];
  //obj_marker.pose.orientation = quat;
  obj_marker.pose.orientation.x = quat.getX();
  obj_marker.pose.orientation.y = quat.getY();
  obj_marker.pose.orientation.z = quat.getZ();
  obj_marker.pose.orientation.w = quat.getW();

  text_marker.pose.orientation.x = 0.0;//quat.getX();
  text_marker.pose.orientation.y = 0.0;//quat.getY();
  text_marker.pose.orientation.z = 0.0;//quat.getZ();
  text_marker.pose.orientation.w = quat.getW();

  obj_marker.scale.x = width;
  obj_marker.scale.y = height;
  obj_marker.scale.z = depth;
  text_marker.scale.x = 1.0;
  text_marker.scale.y = 1.0;
  text_marker.scale.z = 0.02;

  // Set the color -- be sure to set alpha to something non-zero!
  obj_marker.color.r = 0.0f;
  obj_marker.color.g = 1.0f;
  obj_marker.color.b = 0.0f;
  obj_marker.color.a = 0.3;

  // Set the color -- be sure to set alpha to something non-zero!
  text_marker.color.r = 1.0f;
  text_marker.color.g = 1.0f;
  text_marker.color.b = 0.0f;
  text_marker.color.a = 0.9;

  text_marker.text = text;

  obj_marker.lifetime = ros::Duration();
  text_marker.lifetime = ros::Duration();
}

void BoundingBox::transform_Markers(Eigen::Matrix4f Tm, PointType centroid)
{
    tf::Matrix3x3 tf3d;
    // Eigen::Transform<Eigen::Matrix3f, Eigen::Affine> t (Tm);
    tf3d.setValue(static_cast<double>(Tm(0,0)), static_cast<double>(Tm(0,1)), static_cast<double>(Tm(0,2)),
            static_cast<double>(Tm(1,0)), static_cast<double>(Tm(1,1)), static_cast<double>(Tm(1,2)),
            static_cast<double>(Tm(2,0)), static_cast<double>(Tm(2,1)), static_cast<double>(Tm(2,2)));
    tf::Quaternion tfqt;
    tf3d.getRotation(tfqt);


    obj_marker.pose.position.x = centroid.x; //centroid[0];
    text_marker.pose.position.x = centroid.x;//centroid[0];
    obj_marker.pose.position.y = centroid.y;//centroid[1];
    text_marker.pose.position.y = min_point_OBB.y;//centroid[1];
    obj_marker.pose.position.z = centroid.z;//centroid[2];
    text_marker.pose.position.z = centroid.z;//centroid[2];

    obj_marker.pose.orientation.x = tfqt.getX();
    obj_marker.pose.orientation.y = tfqt.getY();
    obj_marker.pose.orientation.z = tfqt.getZ();
    obj_marker.pose.orientation.w = tfqt.getW();

    text_marker.pose.orientation.x = 0.0;//quat.getX();
    text_marker.pose.orientation.y = 0.0;//quat.getY();
    text_marker.pose.orientation.z = 0.0;//quat.getZ();
    text_marker.pose.orientation.w = tfqt.getW();

}

//void BoundingBox::remove()
//{
//    delete_marker.action = visualization_msgs::Marker::DELETE;
//}

//float BoundingBox::operator -(const BoundingBox& rhs) const{
  
//  //jump through a few hoops to build up to a 4x4 translation matrix...
//  Translation3f __translation(
//                              centroid.x() - rhs.centroid.x(),
//                              centroid.y() - rhs.centroid.y(),
//                              centroid.z() - rhs.centroid.z());
  
//  Affine3f _translation(__translation);
  
  
  
  
//  _translation *= Scaling(
//                         (rhs.max_point_OBB.x - rhs.min_point_OBB.x)/(max_point_OBB.x - min_point_OBB.x),
//                         (rhs.max_point_OBB.y - rhs.min_point_OBB.y)/(max_point_OBB.y - min_point_OBB.y),
//                         (rhs.max_point_OBB.z - rhs.min_point_OBB.z)/(max_point_OBB.z - min_point_OBB.z));
//  Matrix4f translation = _translation.matrix();
//  //calculate the inverse of `this` bounding box's rotation matrix,
//  // which we can then take and left multiply the other matrix by;
//  // in other words, we want to find the matrix X in
//  //  X * this->rotational_matrix_OBB = rhs.rotational_matrix_OBB
//  // so,
//  //  X = rhs.rotational_matrix_OBB * this->rotational_matrix_OBB.invers()
  
//  Matrix3f invTransform = rotational_matrix_OBB.inverse();
//  Matrix3f _rotateLHStoRHS = rhs.rotational_matrix_OBB * invTransform;
//  Eigen::Matrix4f rotateLHStoRHS = Eigen::Matrix4f::Identity();
//  rotateLHStoRHS.block(0,0,3,3) = _rotateLHStoRHS;
  
//  auto det1 = (rotateLHStoRHS * translation).determinant();
//  return 1.0 - det1;
//}


//BoundingBox GraspableObject::getBoundingBox() const{
//  return BoundingBox(this->pointCloud);
//}
