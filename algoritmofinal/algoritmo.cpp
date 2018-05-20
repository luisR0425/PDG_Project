#include <iostream>
#include <fstream>
#include <string>

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/ply_io.h>
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
#include <vector>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/filters/filter_indices.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/filters/bilateral.h>
#include <pcl/point_types_conversion.h>


class AlgoritmoPDG
{

public:


std::string clasificacion;
float altura;
float ancho;
float profundidad;


void calibracion( pcl::PointCloud<pcl::PointXYZRGB>::Ptr source_cloud, std::string filename){

  
  std::string time;
  std::string X_value;
  std::string Y_value;
  std::string Z_value;

  double X_valuen;
  double Y_valuen;
  double Z_valuen;

  ifstream ip(filename);


  Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();


  getline(ip,time,',');
  getline(ip,X_value,',');
  getline(ip,Y_value,',');
  getline(ip,Z_value,'\n');

  while(ip.good()){

  getline(ip,time,',');
  getline(ip,X_value,',');
  getline(ip,Y_value,',');
  getline(ip,Z_value,'\n');

  X_valuen = stof(X_value);
  Y_valuen = stof(Y_value);
  Z_valuen = stof(Z_value);


  transform_2.rotate (Eigen::AngleAxisf (Z_valuen, Eigen::Vector3f::UnitZ()));
  transform_2.rotate (Eigen::AngleAxisf (X_valuen, Eigen::Vector3f::UnitX()));
  transform_2.rotate (Eigen::AngleAxisf (Y_valuen, Eigen::Vector3f::UnitY()));

  ip.close();
  }

  // Executing the transformation
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());
  pcl::transformPointCloud (*source_cloud, *source_cloud, transform_2);

}

void filtrobilateral( pcl::PointCloud<pcl::PointXYZI>::Ptr cloud){

  pcl::BilateralFilter<pcl::PointXYZI> bFilter;
  bFilter.setInputCloud (cloud);
  bFilter.setHalfSize(80);
  bFilter.setStdDev(80);
  bFilter.filter (*cloud);
 
  std::cerr << "PointCloud after filtering: " << cloud->width * cloud->height
       << " data points (" << pcl::getFieldsList (*cloud) << ").";
 
  pcl::PLYWriter writer;
  writer.write<pcl::PointXYZI>("resultado-b.ply", *cloud);
}

void binarizacion(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_colored){

  float pequeno = 0.04f;
  float mediano1 = 0.7f;
  float mediano2 = 0.11f;
  float grande = 0.17f;
  float factor = 0.0f;

  pcl::PointXYZRGB minPt, maxPt;
  pcl::getMinMax3D (*cloud, minPt, maxPt);
  std::cout << "Max y: " << maxPt.y << std::endl;


  pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
  int red = 255;

  if (maxPt.y<=0.08f){
    factor = pequeno;
  }else if (maxPt.y>0.08f && maxPt.y<=0.12f){
    factor = mediano1;
  }else if (maxPt.y>0.12f && maxPt.y<=0.16f){
    factor = mediano2;
  }else if (maxPt.y>0.16f){
    factor = grande;
  }

  std::cout << "factor: " << factor << std::endl;
  for (int i = 0; i < (*cloud).size(); i++)
  {
    pcl::PointXYZ pt(cloud->points[i].x, cloud->points[i].y, cloud->points[i].z);
    if (abs(maxPt.y-pt.y) < factor){
    inliers->indices.push_back(i);
    cloud_colored->points[i].r = 255;
    cloud_colored->points[i].g = 0;
    cloud_colored->points[i].b = 0;
    }
  }

  pcl::ExtractIndices<pcl::PointXYZRGB> extract;

  extract.setInputCloud (cloud);
  extract.setIndices(inliers);
  extract.setNegative (false);
  extract.filter (*cloud);

  pcl::PLYWriter writer;



  writer.write<pcl::PointXYZRGB>("resultado.ply", *cloud_colored);

}

void boundingbox(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, std::string filename){

  pcl::PCA<pcl::PointXYZRGB> pca;
  pcl::PointCloud<pcl::PointXYZRGB> proj;

  pca.setInputCloud (cloud);
  pca.project (*cloud, proj);

  pcl::PointXYZRGB proj_min;
  pcl::PointXYZRGB proj_max;
  pcl::getMinMax3D (proj, proj_min, proj_max);

  pcl::PointXYZRGB min2;
  pcl::PointXYZRGB max2;
  pca.reconstruct (proj_min, min2);
  pca.reconstruct (proj_max, max2);

  std::cout << " min.x= " << min2.x << " max.x= " << max2.x << " min.y= " <<
  min2.y << " max.y= " << max2.y << " min.z= " << min2.z << " max.z= " << max2.z << std::endl;


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

  std::cout << "Anchura = " << width*100 << " Altura = " << height*100 << " profundidad = " <<
  depth*100 << std::endl;

  if (depth <= 8){
  clasificacion = "Pequeño";
  }else if(depth > 8 && depth <= 16){
  clasificacion = "Mediano";
  }else if(depth > 16){
  clasificacion = "Grande";
  }


  altura = height*100;
  ancho = width*100;
  profundidad = depth*100;

  std::cout << altura << std::endl;




  serializar(filename);

  // boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new
  // pcl::visualization::PCLVisualizer ("3D Viewer"));
  // viewer->setBackgroundColor (0, 0, 0);
  // viewer->addPointCloud<pcl::PointXYZRGB> (cloud, "NAO arm cloud");
  // viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "NAO arm cloud");


  // viewer->addCube (translation, rotation, depth, width, height);

  // viewer->setRepresentationToWireframeForAllActors ();
  // //--------------------
  // // -----Main loop-----
  // //--------------------
  // while (!viewer->wasStopped ())
  // {
  //   viewer->spinOnce (100);
  //   boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  // }
}

void serializar(std::string filename){

  std::ofstream fs;

  std::string time;
  std::string X_value;
  std::string Y_value;
  std::string Z_value;

  std::cout << "entra" << std::endl;

  std::ifstream ip2(filename);

  fs.open("resultado.csv");

  while(ip2.good()){

  getline(ip2,time,',');
  getline(ip2,X_value,',');
  getline(ip2,Y_value,',');
  getline(ip2,Z_value,'\n');

  fs << time << ",";
  fs << X_value << ",";
  fs << Y_value << ",";
  fs << Z_value << "\n";

  }



  fs << "\n";
  fs << "Latitud,";
  fs << "Longitud,";
  fs << "\n";
  fs << "\n";
  fs << "Altura,";
  fs << "Ancho,";
  fs << "Profundidad,";
  fs << "Clasificación \n";
  fs << altura << ",";
  fs << ancho << ",";
  fs << profundidad << ",";
  fs << clasificacion << "\n";

}
};


int main (int argc, char* argv[])
{

  // Load the target cloud PLY file
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_colored (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloudIntensity (new pcl::PointCloud<pcl::PointXYZI>);
  pcl::io::loadPLYFile (argv[1], *cloud);
  pcl::io::loadPLYFile (argv[1], *cloud_colored);


  std::string filename = argv[2];
  


  
  AlgoritmoPDG algoritmo;

  algoritmo.calibracion(cloud_colored, argv[2]);
  algoritmo.binarizacion(cloud, cloud_colored);
  pcl::PointCloudXYZRGBtoXYZI(*cloud_colored, *cloudIntensity);
  algoritmo.filtrobilateral(cloudIntensity);
  algoritmo.boundingbox(cloud, argv[2]);
  //algoritmo.serializar(argv[2]);
  
  return 0;

}
