#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/bilateral.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/io/ascii_io.h>
#include <pcl/impl/point_types.hpp>

 
int
main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI> ());
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZI> ());
 
  // Fill in the cloud data
  pcl::PLYReader reader;
  //pcl::PCDReader reader;
  // Replace the path below with the path where you saved your file
  reader.read ("file.ply", *cloud); // Remember to download the file first!
 
  std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height
       << " data points (" << pcl::getFieldsList (*cloud) << ").";
 
  // Create the filtering object
  pcl::BilateralFilter<pcl::PointXYZI> bFilter;
  bFilter.setInputCloud (cloud);
  bFilter.setHalfSize(1.2);
  bFilter.setStdDev(0.5);
  bFilter.filter (*cloud_filtered);
 
  std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height
       << " data points (" << pcl::getFieldsList (*cloud_filtered) << ").";
 
  pcl::PLYWriter writer;
  writer.write<pcl::PointXYZI>("12-6-1filter.ply", *cloud_filtered);
 
  return (0);
}