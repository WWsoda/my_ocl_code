// 官网教程：http://pointclouds.org/documentation/tutorials/project_inliers.php#project-inliers
// 在官网教程基础上选择实例pcd文件
// 增加点云可视化部分代码
// 增加计时部分代码，使用PCL自带的计时函数计时
 
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>
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
 
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected(new pcl::PointCloud<pcl::PointXYZ>);
	// 填入点云数据
	/* cloud->width  = 5;
	cloud->height = 1;
	cloud->points.resize (cloud->width * cloud->height);
	for (size_t i = 0; i < cloud->points.size (); ++i)
	{
	  cloud->points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
	  cloud->points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
	  cloud->points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);
	}
	std::cerr << "Cloud before projection: " << std::endl;
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
 
	// 创建一个系数为X=Y=0,Z=1的平面
	/* pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
	coefficients->values.resize(4);
	coefficients->values[0] = coefficients->values[1] = 0;
	coefficients->values[2] = 1.0;
	coefficients->values[3] = 0;
	*/
 
	// 二维圆
	/* pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
	coefficients->values.resize(3);
	coefficients->values[0] = coefficients->values[1] = 0;
	coefficients->values[2] = 1.0;
	*/
 
	// 三维直线
	/* pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
	coefficients->values.resize(6);
	coefficients->values[0] = coefficients->values[1] = coefficients->values[2] = 0;
	coefficients->values[3] = coefficients->values[4] = 0; 
	coefficients->values[5] = 1.0;
	*/
 
	// 圆柱体
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
	coefficients->values.resize(7);
	coefficients->values[0] = coefficients->values[1] = coefficients->values[2] = 0;
	coefficients->values[3] = coefficients->values[4] = 0;
	coefficients->values[5] = -1.0;
	coefficients->values[6] = 0.5;
 
	// 创建滤波器对象
	pcl::ProjectInliers<pcl::PointXYZ> proj;
	// proj.setModelType(pcl::SACMODEL_PLANE);
	// proj.setModelType(pcl::SACMODEL_CIRCLE2D); // 二维圆
	// proj.setModelType(pcl::SACMODEL_LINE); // 三维直线
	proj.setModelType(pcl::SACMODEL_CYLINDER); // 圆柱体
	proj.setInputCloud(cloud);
	proj.setModelCoefficients(coefficients);
	proj.filter(*cloud_projected);
 
	/* std::cerr << "Cloud after projection: " << std::endl;
	for (size_t i = 0; i < cloud_projected->points.size (); ++i)
	  std::cerr << "    " << cloud_projected->points[i].x << " "
						  << cloud_projected->points[i].y << " "
						  << cloud_projected->points[i].z << std::endl;
						  */
	std::stringstream str2;
	str2 << "PointCloud after filtering: " << cloud_projected->width * cloud_projected->height
		<< " data points (" << pcl::getFieldsList(*cloud_projected) << ").";
 
	pcl::PCDWriter writer;
	writer.write("table_scene_lms400_projected.pcd", *cloud_projected);
 
	std::cout << "运行时间:" << time.getTime() << "ms（毫秒）" << std::endl;
	std::cout << "运行时间:" << time.getTimeSeconds() << "s（秒）" << std::endl;
 
	// 可视化
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("project_inliers 3D Viewer"));
	viewer->initCameraParameters(); // 通过设置照相机参数使得从默认的角度和方向观察点云
 
	int v1(0);  // 创建新的视口
	viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1); // 4个参数分别是X轴的最小值，最大值，Y轴的最小值，最大值，取值0-1，v1是标识
	viewer->setBackgroundColor(0, 0, 0, v1); // 设置视窗的背景颜色
	// viewer->addText("cloud before filtering", 10, 10, "v1 text", v1); // 添加一个标签区别其他窗口
	viewer->addText(str1.str(), 10, 10, "v1 text", v1); // 添加一个标签区别其他窗口
	viewer->addPointCloud<pcl::PointXYZ>(cloud, "cloud", v1);
 
	// 对第二视口做同样的操作，使得做创建的点云分布于右半窗口，将该视口背景赋值于灰色，以便明显区别，虽然添加同样的点云，给点云自定义颜色着色
	int v2(0);
	viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
	viewer->setBackgroundColor(0.3, 0.3, 0.3, v2);
	// viewer->addText("cloud after filtering", 10, 10, "v2 text", v2);
	viewer->addText(str2.str(), 10, 10, "v2 text", v2);
	viewer->addPointCloud<pcl::PointXYZ>(cloud_projected, "cloud_projected", v2);
	// 为所有视口设置属性
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud_projected");
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
