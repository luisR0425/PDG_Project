#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/surface/reconstruction.h>
#include <pcl/common/pca.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/voxel_grid_covariance.h>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/parse.h>

int
main (int argc, char* argv[])
{

  if (argc < 2)
  {
    printf ("No target PCD file given!\n");
    return (-1);
  }

  // Load the target cloud PCD file
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::io::loadPLYFile (argv[1], *cloud);
  std::cout << "PointCloud before filtering has: " << cloud->points.size () 
  << " data points." << std::endl; //*

  pcl::PCA<pcl::PointXYZ> pca;
  pcl::PointCloud<pcl::PointXYZ> proj;

  pca.setInputCloud (cloud);
  pca.project (*cloud, proj);

  pcl::PointXYZ proj_min;
  pcl::PointXYZ proj_max;
  pcl::getMinMax3D (proj, proj_min, proj_max);

  pcl::PointXYZ min;
  pcl::PointXYZ max;
  //pca.reconstruct (proj_min, min);
  //pca.reconstruct (proj_max, max);
  std::cout << " min.x= " << min.x << " max.x= " << max.x << " min.y= " <<
min.y << " max.y= " << max.y << " min.z= " << min.z << " max.z= " << max.z
<< std::endl;

  //Rotation of PCA
  Eigen::Matrix3f rot_mat = pca.getEigenVectors ();

  //translation of PCA
  Eigen::Vector3f cl_translation = pca.getMean().head(3);

  Eigen::Matrix3f affine_trans;
  std::cout << rot_mat << std::endl;
  //Reordering of principal components
  affine_trans.col(0) << (rot_mat.col(0).cross(rot_mat.col(1))).normalized();
  affine_trans.col(1) << rot_mat.col(0);
  affine_trans.col(2) << rot_mat.col(1);
  //affine_trans.col(3) << cl_translation,1;/**/

  std::cout << affine_trans << std::endl;

  Eigen::Quaternionf rotation = Eigen::Quaternionf (affine_trans);
  Eigen::Vector4f t = pca.getMean();

  Eigen::Vector3f translation = Eigen::Vector3f (t.x(), t.y(), t.z());

  double width = fabs(proj_max.x-proj_min.x);
  double height = fabs(proj_max.y-proj_min.y);
  double depth = fabs(proj_max.z-proj_min.z);

  //adding the bounding box to a viewer :
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new
pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (200, 0, 0);
  viewer->addPointCloud<pcl::PointXYZ> (cloud, "NAO arm cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "NAO arm cloud");
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  //viewer->addCube (min.x, max.x, min.y, max.y, min.z, max.z);/**/
  viewer->addCube (translation, rotation, width, height, depth);
  //--------------------
  // -----Main loop-----
  //--------------------
  while (!viewer->wasStopped ())
  {
    viewer->spinOnce (100);
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }

  return 0;

}