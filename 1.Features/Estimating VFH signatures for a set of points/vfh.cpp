#include <pcl/point_types.h>
#include <pcl/features/vfh.h>
 
 {
   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
   pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal> ());
 
  /*
		 此处通过读取PCD或者创造一组有法向特征的点云。并使上面的指针指向点云，不再赘述
 	*/

  // 创造VFH估计类，并且将点云数据和法向输出传入。
  pcl::VFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::VFHSignature308> vfh;
  vfh.setInputCloud (cloud);
  vfh.setInputNormals (normals);
  // 当点云格式是PointNormal时，也可选择使用语句vfh.setInputNormals (cloud);

  // 创建空KD树，并将VFH类设置成树状结构便于快速搜索
  // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
  vfh.setSearchMethod (tree);

  // 输出数据的指针
  pcl::PointCloud<pcl::VFHSignature308>::Ptr vfhs (new pcl::PointCloud<pcl::VFHSignature308> ());

  // 计算VFH特征
  vfh.compute (*vfhs);

  // 如上所述此时vfhs->size()应该为1，这是与PFH和FPFH的区别
}
