// 官网教程：http://pointclouds.org/documentation/tutorials/statistical_outlier.php#statistical-outlier-removal
// 在官网教程基础上选择实例pcd文件
// 增加点云可视化部分代码，实例pcd文件为XYZI格式，可视化颜色根据pcd文件"intensity"属性生成
// 增加计时部分代码，使用PCL自带的计时函数计时
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/common/time.h> // 计时
// 可视化相关头文件都一起堆在这儿
#include <thread>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
 
int
main(int argc, char** argv)
{
	pcl::StopWatch time; // 计时开始
 
	// pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	// pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>);
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_inliers(new pcl::PointCloud<pcl::PointXYZI>);
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_outliers(new pcl::PointCloud<pcl::PointXYZI>);
 
	// Fill in the cloud data
	pcl::PCDReader reader;
	// Replace the path below with the path where you saved your file
	reader.read<pcl::PointXYZI>("table_scene_lms400.pcd", *cloud);
	std::stringstream str1;
	str1 << "PointCloud before filtering: " << cloud->width * cloud->height
		<< " data points (" << pcl::getFieldsList(*cloud) << ").";
 
	std::cerr << "Cloud before filtering: " << std::endl;
	std::cerr << *cloud << std::endl;
 
	// Create the filtering object
	pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;
	sor.setInputCloud(cloud);
	sor.setMeanK(50);
	sor.setStddevMulThresh(1.0);
	sor.filter(*cloud_filtered);
 
	std::cerr << "Cloud after filtering: " << std::endl;
	std::cerr << *cloud_filtered << std::endl;
 
	pcl::PCDWriter writer;
	writer.write<pcl::PointXYZI>("table_scene_lms400_inliers.pcd", *cloud_filtered, false);
 
	sor.setNegative(true);
	sor.filter(*cloud_filtered);
	writer.write<pcl::PointXYZI>("table_scene_lms400_outliers.pcd", *cloud_filtered, false);
	
	std::cout << "运行时间:" << time.getTime() << "ms（毫秒）" << std::endl;
	std::cout << "运行时间:" << time.getTimeSeconds() << "s（秒）" << std::endl;
 
	// 可视化
	reader.read<pcl::PointXYZI>("./table_scene_lms400_inliers.pcd", *cloud_inliers);
	reader.read<pcl::PointXYZI>("./table_scene_lms400_outliers.pcd", *cloud_outliers);
	std::stringstream str2;
	str2 << "inliersPointCloud after filtering: " << cloud_inliers->width * cloud_inliers->height
		<< " data points (" << pcl::getFieldsList(*cloud_inliers) << ").";
	std::stringstream str3;
	str3 << "outliersPointCloud after filtering: " << cloud_outliers->width * cloud_outliers->height
		<< " data points (" << pcl::getFieldsList(*cloud_outliers) << ").";
 
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("statistical_removal 3D Viewer"));
	viewer->initCameraParameters(); // 通过设置照相机参数使得从默认的角度和方向观察点云
	
	int v1(0);  // 创建新的视口
	viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1); // 4个参数分别是X轴的最小值，最大值，Y轴的最小值，最大值，取值0-1，v1是标识
	viewer->setBackgroundColor(0, 0, 0, v1); // 设置视窗的背景颜色
	viewer->addText(str1.str(), 10, 10, "v1 text", v1); // 添加一个标签区别其他窗口
	pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI>cloud_color(cloud, "intensity");
	viewer->addPointCloud<pcl::PointXYZI>(cloud, cloud_color, "cloud", v1);
 
	// 对第二视口做同样的操作，使得做创建的点云分布于右半窗口，将该视口背景赋值于灰色，以便明显区别，虽然添加同样的点云，给点云自定义颜色着色
	int v2(0);
	viewer->createViewPort(0.5, 0.5, 1.0, 1.0, v2);
	viewer->setBackgroundColor(0.3, 0.3, 0.3, v2);
	viewer->addText(str2.str(), 10, 10, "v2 text", v2);
	pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI>cloud_inliers_color(cloud_inliers, "intensity");
	viewer->addPointCloud<pcl::PointXYZI>(cloud_inliers, cloud_inliers_color, "cloud_inliers", v2);
 
	int v3(0);
	viewer->createViewPort(0.5, 0.0, 1.0, 0.5, v3);
	viewer->setBackgroundColor(0.5, 0.5, 0.5, v3);
	viewer->addText(str3.str(), 10, 10, "v3 text", 3);
	pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI>cloud_outliers_color(cloud_outliers, "intensity");
	viewer->addPointCloud<pcl::PointXYZI>(cloud_outliers, cloud_outliers_color, "cloud_outliers", v3);
 
	// 为所有视口设置属性
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud_inliers");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud_outliers");
	//  改变显示点云的尺寸
	viewer->addCoordinateSystem(1.0); // 从默认的角度和方向观察点云
 
	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		//boost::this_thread::sleep(boost::posix_time::microseconds(10000));
	} // 每次调用spinOnce都给视窗处理事件的时间，允许鼠标、键盘等交互操作
 
	getchar();
	return (0);
}
