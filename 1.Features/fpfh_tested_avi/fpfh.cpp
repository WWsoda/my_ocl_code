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

// This function displays the help
void showHelp(char* program_name)
{
	std::cout << std::endl;
	std::cout << "Usage: " << program_name << " cloud_filename.[pcd|ply]" << std::endl;
	std::cout << "-h or --help :  Show  help." << std::endl;
}

using namespace std;
typedef pcl::PointCloud<pcl::PointXYZ> pointcloud;
typedef pcl::PointCloud<pcl::Normal> pointnormal;
typedef pcl::PointCloud<pcl::FPFHSignature33> fpfhFeature;

fpfhFeature::Ptr compute_fpfh_feature(pointcloud::Ptr input_cloud, pcl::search::KdTree<pcl::PointXYZ>::Ptr tree)
{
	//法向量
	pointnormal::Ptr point_normal(new pointnormal);
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> est_normal;
	est_normal.setInputCloud(input_cloud);
	est_normal.setSearchMethod(tree);
	est_normal.setKSearch(20);
	est_normal.compute(*point_normal);
	//fpfh 估计
	fpfhFeature::Ptr fpfh(new fpfhFeature);
	//pcl::FPFHEstimation<pcl::PointXYZ,pcl::Normal,pcl::FPFHSignature33> est_target_fpfh;
	pcl::FPFHEstimationOMP<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> est_fpfh;
	est_fpfh.setNumberOfThreads(8); //指定4核计算
	// pcl::search::KdTree<pcl::PointXYZ>::Ptr tree4 (new pcl::search::KdTree<pcl::PointXYZ> ());
	est_fpfh.setInputCloud(input_cloud);
	est_fpfh.setInputNormals(point_normal);
	est_fpfh.setSearchMethod(tree);
	est_fpfh.setKSearch(10);
	est_fpfh.compute(*fpfh);

	return fpfh;

}

// This is the main function
int main(int argc, char** argv)
{
	pcl::console::TicToc time; time.tic();
	// Show help
	if (pcl::console::find_switch(argc, argv, "-h") || pcl::console::find_switch(argc, argv, "--help")) {
		std::cout << "没有help." << std::endl;
		return 0;
	}

	// Fetch point cloud filename in arguments | Works with PCD and PLY files
	std::vector<int> filenames;
	bool file_is_pcd = false;

	filenames = pcl::console::parse_file_extension_argument(argc, argv, ".ply");

	if (filenames.size() != 1) {
		filenames = pcl::console::parse_file_extension_argument(argc, argv, ".pcd");

		if (filenames.size() != 1) {
			showHelp(argv[0]);
			return -1;
		}
		else {
			file_is_pcd = true;
		}
	}

	// Load file | Works with PCD and PLY files
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr source(new pcl::PointCloud<pcl::PointXYZ>());
	if (file_is_pcd) {
		if (pcl::io::loadPCDFile(argv[filenames[0]], *cloud) < 0) {
			std::cout << "Error loading point cloud " << argv[filenames[0]] << std::endl << std::endl;
			showHelp(argv[0]);
			return -1;
		}
	}
	else {
		if (pcl::io::loadPLYFile(argv[filenames[0]], *cloud) < 0) {
			std::cout << "Error loading point cloud " << argv[filenames[0]] << std::endl << std::endl;
			showHelp(argv[0]);
			return -1;
		}
	}
	std::cout << "cloud size is: " << cloud->size() << std::endl;

	//
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	fpfhFeature::Ptr cloud_fpfh = compute_fpfh_feature(cloud, tree);
	std::cout << "直方图计算结束" << std::endl << std::endl;

	//1
	pcl::visualization::PCLPlotter plotter;
	// We need to set the size of the descriptor beforehand.
	plotter.addFeatureHistogram(*cloud_fpfh, 300); //设置的heng坐标长度，该值越大，则显示的越细致
	plotter.plot();

	cout << "代码段运行时间: " << time.toc() / 1000 << "s" << endl;//计算程序运行时间

	//system("pause");//不注释，命令行会提示按任意键继续，注释会直接跳出
	return 0;
}

