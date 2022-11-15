#pragma once
#include <pcl/features/rops_estimation.h>
#include <pcl/io/pcd_io.h>

#include <pcl/console/time.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/histogram_visualizer.h> //直方图的可视化

#include <pcl/point_cloud.h>
#include <pcl/features/fpfh.h>

#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <pcl/features/fpfh_omp.h> //包含fpfh加速计算的omp(多核并行计算)
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_features.h> //特征的错误对应关系去除
#include <pcl/registration/correspondence_rejection_sample_consensus.h> //随机采样一致性去除
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/features/integral_image_normal.h>
#include <iostream>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>
#include <pcl/visualization/pcl_plotter.h>
// 包含相关头文件


int main (int argc, char** argv)
{
  if (argc != 4)
    return (-1);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ> ());
  if (pcl::io::loadPCDFile (argv[1], *cloud) == -1)
    return (-1);

  pcl::PointIndicesPtr indices (new pcl::PointIndices);
  std::ifstream indices_file;
  indices_file.open (argv[2], std::ifstream::in);
  for (std::string line; std::getline (indices_file, line);)
  {
    std::istringstream in (line);
    unsigned int index = 0;
    in >> index;
    indices->indices.push_back (index - 1);
  }
  indices_file.close ();

  std::vector <pcl::Vertices> triangles;
  std::ifstream triangles_file;
  triangles_file.open (argv[3], std::ifstream::in);
  for (std::string line; std::getline (triangles_file, line);)
  {
    pcl::Vertices triangle;
    std::istringstream in (line);
    unsigned int vertex = 0;
    in >> vertex;
    triangle.vertices.push_back (vertex - 1);
    in >> vertex;
    triangle.vertices.push_back (vertex - 1);
    in >> vertex;
    triangle.vertices.push_back (vertex - 1);
    triangles.push_back (triangle);
  }

  float support_radius = 0.0285f;
  unsigned int number_of_partition_bins = 5;
  unsigned int number_of_rotations = 3;

  pcl::search::KdTree<pcl::PointXYZ>::Ptr search_method (new pcl::search::KdTree<pcl::PointXYZ>);
  search_method->setInputCloud (cloud);

  pcl::ROPSEstimation <pcl::PointXYZ, pcl::Histogram <135> > feature_estimator;
  feature_estimator.setSearchMethod (search_method);
  feature_estimator.setSearchSurface (cloud);
  feature_estimator.setInputCloud (cloud);
  feature_estimator.setIndices (indices);
  feature_estimator.setTriangles (triangles);
  feature_estimator.setRadiusSearch (support_radius);
  feature_estimator.setNumberOfPartitionBins (number_of_partition_bins);
  feature_estimator.setNumberOfRotations (number_of_rotations);
  feature_estimator.setSupportRadius (support_radius);

  pcl::PointCloud<pcl::Histogram <135> >::Ptr histograms (new pcl::PointCloud <pcl::Histogram <135> > ());
  feature_estimator.compute (*histograms);

 // ========直方图可视化=============================
      pcl::visualization::PCLPlotter plotter;
      plotter.addFeatureHistogram(*histograms, 300); //设置的很坐标长度，该值越大，则显示的越细致
      plotter.plot();


  return (0);
}
