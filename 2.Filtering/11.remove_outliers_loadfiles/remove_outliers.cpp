#include <iostream>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/visualization/pcl_visualizer.h>
 
int main(int argc,char** argv) {
    std::cout << "Hello, World!" << std::endl;
 
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_conditional (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_radius(new pcl::PointCloud<pcl::PointXYZ>);
 
    pcl::io::loadPCDFile("room_scan1.pcd",*cloud_in);
    //radius_remove
    pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
    outrem.setInputCloud(cloud_in);
    outrem.setRadiusSearch(0.5);
    outrem.setMinNeighborsInRadius(20);
    outrem.filter(*cloud_radius);
    pcl::io::savePCDFileASCII("cloud_radius.pcd",*cloud_radius);
 
    //condition_remove 的滤波器
    //build the filter
    pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_cond(new pcl::ConditionAnd<pcl::PointXYZ>);//实例化条件指针
    range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ>("z",pcl::ComparisonOps::GT,0.0)));
    range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZ>("z",pcl::ComparisonOps::LT,1.0)));
    //build the filter
    pcl::ConditionalRemoval<pcl::PointXYZ> condrem;
    condrem.setCondition(range_cond);
    condrem.setInputCloud(cloud_in);
    condrem.setKeepOrganized(true);//保存原有点云结结构就是点的数目没有减少，采用nan代替了
    //apply filter
    condrem.filter(*cloud_conditional);
    pcl::io::savePCDFileASCII("cloud_conditional.pcd",*cloud_conditional);
 
 
    std::cerr<<"cloud_in: "<<cloud_in->size()<<" points"<<std::endl;
    std::cerr<<"cloud_radius: "<<cloud_radius->size()<<" points"<<std::endl;
    std::cerr<<"cloud_conditional: "<<cloud_conditional->size()<<" points"<<std::endl;
    //visualizer
 
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer);
    viewer->initCameraParameters();
 
    int v1(0);
    viewer->createViewPort(0,0,0.33,1,v1);
    viewer->setBackgroundColor(0,255,0,v1);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color1(cloud_in,255,0,0);
    viewer->addPointCloud(cloud_in,color1,"cloud_in",v1);//C++赋值兼容规则。派生类对象可以用来初始化基类的引用
 
    int v2(0);
    viewer->createViewPort(0.33,0,0.66,1,v2);
    viewer->setBackgroundColor(0,0,255,v2);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color2(cloud_radius,255,0,0);
    viewer->addPointCloud(cloud_radius,color2,"cloud_radius",v2);
 
    int v3(0);
    viewer->createViewPort(0.66,0,1,1,v3);
    viewer->setBackgroundColor(0,255,0,v3);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color3(cloud_conditional,255,0,0);
    viewer->addPointCloud(cloud_conditional,color3,"cloud_conditional",v3);
 
    viewer->addCoordinateSystem();
 
    viewer->spin();
 
 
    return 0;
}
