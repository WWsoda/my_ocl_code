// 官网教程：http://pointclouds.org/documentation/tutorials/passthrough.php#passthrough
// 在官网教程基础上选择实例pcd文件
// 增加点云可视化部分代码，实例pcd文件为XYZI格式，可视化颜色根据pcd文件"intensity"属性生成
// 增加计时部分代码，使用PCL自带的计时函数计时
 
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <ctime>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/time.h> // 计时
// 可视化相关头文件都一起堆在这儿
#include <thread>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <boost/thread/thread.hpp>
 
int
main(int argc, char** argv)
{
	// srand(time(0));
	pcl::StopWatch time; // 计时开始
 
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>);
	//填入点云数据
	/* cloud->width  = 5;
	cloud->height = 1;
	cloud->points.resize (cloud->width * cloud->height);
	for (size_t i = 0; i < cloud->points.size (); ++i)
	{
	  cloud->points[i].x = rand () / (RAND_MAX + 1.0f)-0.5;
	  cloud->points[i].y = rand () / (RAND_MAX + 1.0f)-0.5;
	  cloud->points[i].z = rand () / (RAND_MAX + 1.0f)-0.5;
	}
	std::cerr << "Cloud before filtering: " << std::endl;
	for (size_t i = 0; i < cloud->points.size (); ++i)
	  std::cerr << "    " << cloud->points[i].x << " "
						  << cloud->points[i].y << " "
						  << cloud->points[i].z << std::endl;
						  */
 
	pcl::PCDReader reader;
	reader.read("table_scene_lms400.pcd", *cloud);
	std::stringstream str1;
	str1 << "PointCloud before filtering: " << cloud->width * cloud->height
		<< " data points (" << pcl::getFieldsList(*cloud) << ").";
 
	// 创建滤波器对象
	pcl::PassThrough<pcl::PointXYZI> pass;
	pass.setInputCloud(cloud);
	pass.setFilterFieldName("z");
	pass.setFilterLimits(-1.5, 1.0);
	// pass.setFilterFieldName("intensity");
	// pass.setFilterLimits(0.0, 15.0);
	pass.setFilterLimitsNegative(true); // 设置保留范围内的（true）还是过滤掉范围内的（false）
	// pass.setFilterLimitsNegative(false);
	pass.filter(*cloud_filtered);
 
	/* std::cerr << "Cloud after filtering: " << std::endl;
	for (size_t i = 0; i < cloud_filtered->points.size (); ++i)
	  std::cerr << "    " << cloud_filtered->points[i].x << " "
						  << cloud_filtered->points[i].y << " "
						  << cloud_filtered->points[i].z << std::endl;
						  */
 
	std::stringstream str2;
 
	str2 << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height
		<< " data points (" << pcl::getFieldsList(*cloud_filtered) << ").";
	pcl::PCDWriter writer;
	writer.write("table_scene_lms400_passthrough.pcd", *cloud_filtered);
 
	std::cout << "运行时间:" << time.getTime() << "ms（毫秒）" << std::endl;
	std::cout << "运行时间:" << time.getTimeSeconds() << "s（秒）" << std::endl;
 
	// 可视化
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("passthrough 3D Viewer"));
	viewer->initCameraParameters(); // 通过设置照相机参数使得从默认的角度和方向观察点云
 
	int v1(0);  // 创建新的视口
	viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1); // 4个参数分别是X轴的最小值，最大值，Y轴的最小值，最大值，取值0-1，v1是标识
	viewer->setBackgroundColor(0, 0, 0, v1); // 设置视窗的背景颜色
	viewer->addText(str1.str(), 10, 10, "v1 text", v1); // 添加一个标签区别其他窗口
	pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI>cloud_color(cloud, "intensity");
	viewer->addPointCloud<pcl::PointXYZI>(cloud, cloud_color, "cloud", v1);
 
	// 对第二视口做同样的操作，使得做创建的点云分布于右半窗口，将该视口背景赋值于灰色，以便明显区别，虽然添加同样的点云，给点云自定义颜色着色
	int v2(0);
	viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
	viewer->setBackgroundColor(0.3, 0.3, 0.3, v2);
	viewer->addText(str2.str(), 10, 10, "v2 text", v2);
	pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI>cloud_filtered_color(cloud_filtered, "intensity");
	viewer->addPointCloud<pcl::PointXYZI>(cloud_filtered, cloud_filtered_color, "cloud_filtered", v2);
	// 为所有视口设置属性
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud_filtered");
	//  改变显示点云的尺寸
	viewer->addCoordinateSystem(1.0); // 从默认的角度和方向观察点云
 
	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		//boost::this_thread::sleep(boost::posix_time::microseconds(10000));
	} // 每次调用spinOnce都给视窗处理事件的时间，允许鼠标、键盘等交互操作
 
	getchar(); // 防止闪退
	return (0);
}
