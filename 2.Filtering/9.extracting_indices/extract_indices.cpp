#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
 
int main (int argc, char** argv)
{
  pcl::PCLPointCloud2::Ptr cloud_blob (new pcl::PCLPointCloud2), 
   cloud_filtered_blob (new pcl::PCLPointCloud2);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>),
   cloud_p (new pcl::PointCloud<pcl::PointXYZ>), cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
 
  // step1:加载点云PCD文件
  pcl::PCDReader reader;
  reader.read ("table_scene_lms400.pcd", *cloud_blob);
 
  std::cerr << "PointCloud before filtering: " << cloud_blob->width * cloud_blob->height
   << " data points." << std::endl;
 
  // Step2： 创建体素网格对象，并设置网格大小为1cm
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud (cloud_blob);
  sor.setLeafSize (0.01f, 0.01f, 0.01f);
  sor.filter (*cloud_filtered_blob);
 
  // Step3： 将降采样后的点云类型转化为标准点云类型
  pcl::fromPCLPointCloud2 (*cloud_filtered_blob, *cloud_filtered);
 
  std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height 
  << " data points." << std::endl;
 
  // Write the downsampled version to disk
  pcl::PCDWriter writer;
  writer.write<pcl::PointXYZ> ("table_scene_lms400_downsampled.pcd", *cloud_filtered, false);
 
  /**********************************************************************************************************
   以上过程为：从输入的.PCD 文件载入数据后，创建一个VoxelGrid滤波器对数据进行下采样，
   在这里进行下才样是为了加速处理过程，
   越少的点意味着分割循环中处理起来越快
   **********************************************************************************************************/
 
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg; //创建分割对象
  // Optional
  seg.setOptimizeCoefficients (true);      //设置对估计模型参数进行优化处理
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);  //设置分割模型类别
  seg.setMethodType (pcl::SAC_RANSAC);     //设置用哪个随机参数估计方法
  seg.setMaxIterations (1000);            //设置最大迭代次数
  seg.setDistanceThreshold (0.01);        //判断是否为模型内点的距离阀值
 
  // Create the filtering object
  pcl::ExtractIndices<pcl::PointXYZ> extract;  //创建点云提取对象
 
  int i = 0, nr_points = (int) cloud_filtered->size ();
  // While 30% of the original cloud is still there
  // 为了处理点云包含的多个模型，在一个循环中执行该过程并在每次模型被提取后，保存剩余的点进行迭代
  while (cloud_filtered->size () > 0.3 * nr_points)
  {
    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud (cloud_filtered);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
      std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
      break;
    }
 
    // Extract the inliers
    extract.setInputCloud (cloud_filtered);
    extract.setIndices (inliers);
    extract.setNegative (false);
    extract.filter (*cloud_p);
    std::cerr << "PointCloud representing the planar component: " << cloud_p->width * cloud_p->height
     << " data points." << std::endl;
 
    std::stringstream ss;
    ss << "table_scene_lms400_plane_" << i << ".pcd";
    writer.write<pcl::PointXYZ> (ss.str (), *cloud_p, false);
 
    // Create the filtering object
    extract.setNegative (true);
    extract.filter (*cloud_f);
    cloud_filtered.swap (cloud_f);
    i++;
  }
 
  return (0);
}
