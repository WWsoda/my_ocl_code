  /* \作者: Bastian Steder */
  
 #include <iostream>
  
 #include <pcl/range_image/range_image.h> // 深度图头文件
 #include <pcl/io/pcd_io.h>               // PCD文件读取头
 #include <pcl/visualization/range_image_visualizer.h>// 深度图可视化头
 #include <pcl/visualization/pcl_visualizer.h>        // 点云可视化头
 #include <pcl/features/range_image_border_extractor.h>// 深度图边缘提取头
 #include <pcl/keypoints/narf_keypoint.h> // NARF关键点计算头
 #include <pcl/features/narf_descriptor.h>// NARF描述子头
 #include <pcl/console/parse.h>           // 命令行解析头
 #include <pcl/common/file_io.h> // 用于获取没有拓展名的文件
 
 typedef pcl::PointXYZ PointType;
 
 // --------------------
 // -----NAFR的一些-----
 // --------------------
 float angular_resolution = 0.5f; // 角度分辨率
 float support_size = 0.2f; // 关键点计算范围（计算范围球的半径）
 pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME; // 深度图坐标系
 bool setUnseenToMaxRange = false;  // 不设置深度范围
 bool rotation_invariant = true;	// 是否保持旋转不变性
 
 // --------------
 // -----输出帮助信息-----
// --------------
void 
 printUsage (const char* progName)
 {
   std::cout << "\n\nUsage: "<<progName<<" [options] <scene.pcd>\n\n"
             << "Options:\n"
             << "-------------------------------------------\n"
             << "-r <float>   angular resolution in degrees (default "<<angular_resolution<<")\n"
             << "-c <int>     coordinate frame (default "<< (int)coordinate_frame<<")\n"
             << "-m           Treat all unseen points to max range\n"
             << "-s <float>   support size for the interest points (diameter of the used sphere - "
                                                                   "default "<<support_size<<")\n"
             << "-o <0/1>     switch rotational invariant version of the feature on/off"
             <<               " (default "<< (int)rotation_invariant<<")\n"
             << "-h           this help\n"
             << "\n\n";
 }
 
 void  // 设置可视化相机位姿
 setViewerPose (pcl::visualization::PCLVisualizer& viewer, const Eigen::Affine3f& viewer_pose)
 {
   Eigen::Vector3f pos_vector = viewer_pose * Eigen::Vector3f (0, 0, 0);
   Eigen::Vector3f look_at_vector = viewer_pose.rotation () * Eigen::Vector3f (0, 0, 1) + pos_vector;
   Eigen::Vector3f up_vector = viewer_pose.rotation () * Eigen::Vector3f (0, -1, 0);
   viewer.setCameraPosition (pos_vector[0], pos_vector[1], pos_vector[2],
                             look_at_vector[0], look_at_vector[1], look_at_vector[2],
                             up_vector[0], up_vector[1], up_vector[2]);
 }
 
 // --------------
 // -----Main-----
 // --------------
 int 
 main (int argc, char** argv)
 {
   // --------------------------------------
   // -----Parse Command Line Arguments-----
   // --------------------------------------
   if (pcl::console::find_argument (argc, argv, "-h") >= 0)
   {
     printUsage (argv[0]);
     return 0;
   }
   if (pcl::console::find_argument (argc, argv, "-m") >= 0)
   {
     setUnseenToMaxRange = true;
     std::cout << "Setting unseen values in range image to maximum range readings.\n";
   }
   if (pcl::console::parse (argc, argv, "-o", rotation_invariant) >= 0)
     std::cout << "Switching rotation invariant feature version "<< (rotation_invariant ? "on" : "off")<<".\n";
   int tmp_coordinate_frame;
   if (pcl::console::parse (argc, argv, "-c", tmp_coordinate_frame) >= 0)
   {
     coordinate_frame = pcl::RangeImage::CoordinateFrame (tmp_coordinate_frame);
     std::cout << "Using coordinate frame "<< (int)coordinate_frame<<".\n";
   }
   if (pcl::console::parse (argc, argv, "-s", support_size) >= 0)
     std::cout << "Setting support size to "<<support_size<<".\n";
   if (pcl::console::parse (argc, argv, "-r", angular_resolution) >= 0)
     std::cout << "Setting angular resolution to "<<angular_resolution<<"deg.\n";
   angular_resolution = pcl::deg2rad (angular_resolution);
   
   // ------------------------------------------------------------------
   // -----读入PCD点云文件或创造出一些点云-----
   // ------------------------------------------------------------------
   pcl::PointCloud<PointType>::Ptr point_cloud_ptr (new pcl::PointCloud<PointType>);
   pcl::PointCloud<PointType>& point_cloud = *point_cloud_ptr;
   pcl::PointCloud<pcl::PointWithViewpoint> far_ranges;
   Eigen::Affine3f scene_sensor_pose (Eigen::Affine3f::Identity ());
   std::vector<int> pcd_filename_indices = pcl::console::parse_file_extension_argument (argc, argv, "pcd");
   if (!pcd_filename_indices.empty ())
   {
    std::string filename = argv[pcd_filename_indices[0]];
    if (pcl::io::loadPCDFile (filename, point_cloud) == -1)
    {
      std::cerr << "Was not able to open file \""<<filename<<"\".\n";
      printUsage (argv[0]);
      return 0;
    }
    scene_sensor_pose = Eigen::Affine3f (Eigen::Translation3f (point_cloud.sensor_origin_[0],
                                                               point_cloud.sensor_origin_[1],
                                                               point_cloud.sensor_origin_[2])) *
                        Eigen::Affine3f (point_cloud.sensor_orientation_);
    std::string far_ranges_filename = pcl::getFilenameWithoutExtension (filename)+"_far_ranges.pcd";
    if (pcl::io::loadPCDFile (far_ranges_filename.c_str (), far_ranges) == -1)
      std::cout << "Far ranges file \""<<far_ranges_filename<<"\" does not exists.\n";
  }
  else
  {
    setUnseenToMaxRange = true;
    std::cout << "\nNo *.pcd file given => Generating example point cloud.\n\n";
    for (float x=-0.5f; x<=0.5f; x+=0.01f)
    {
      for (float y=-0.5f; y<=0.5f; y+=0.01f) 
           {
        PointType point;  point.x = x;  point.y = y;  point.z = 2.0f - y;
        point_cloud.push_back (point);
      }
    }
    point_cloud.width = point_cloud.size ();  point_cloud.height = 1;
  }
  
  // -----------------------------------------------
  // -----利用点云创造深度图-----
  // -----------------------------------------------
  float noise_level = 0.0;
  float min_range = 0.0f;
  int border_size = 1;
  pcl::RangeImage::Ptr range_image_ptr (new pcl::RangeImage);
  pcl::RangeImage& range_image = *range_image_ptr;   
  range_image.createFromPointCloud (point_cloud, angular_resolution, pcl::deg2rad (360.0f), pcl::deg2rad (180.0f),
                                   scene_sensor_pose, coordinate_frame, noise_level, min_range, border_size);
  range_image.integrateFarRanges (far_ranges);
  if (setUnseenToMaxRange)
    range_image.setUnseenToMaxRange ();
  
  // --------------------------------------------
  // -----打开可视化器并显示点云-----
  // -------------------------------------------- 
  pcl::visualization::PCLVisualizer viewer ("3D Viewer");
  viewer.setBackgroundColor (1, 1, 1);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointWithRange> range_image_color_handler (range_image_ptr, 0, 0, 0);
  viewer.addPointCloud (range_image_ptr, range_image_color_handler, "range image");
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "range image");
  //viewer.addCoordinateSystem (1.0f, "global");
  //PointCloudColorHandlerCustom<PointType> point_cloud_color_handler (point_cloud_ptr, 150, 150, 150);
  //viewer.addPointCloud (point_cloud_ptr, point_cloud_color_handler, "original point cloud");
  viewer.initCameraParameters ();
  setViewerPose (viewer, range_image.getTransformationToWorldSystem ());
  
  // --------------------------
  // -----显示深度图-----
  // --------------------------
  pcl::visualization::RangeImageVisualizer range_image_widget ("Range image");
 range_image_widget.showRangeImage (range_image);
  
  // --------------------------------
 // -----提取NARF特征-----
  // --------------------------------
  pcl::RangeImageBorderExtractor range_image_border_extractor;  
 pcl::NarfKeypoint narf_keypoint_detector; 
 narf_keypoint_detector.setRangeImageBorderExtractor (&range_image_border_extractor);//深度图像边缘点提取
  narf_keypoint_detector.setRangeImage (&range_image);
  narf_keypoint_detector.getParameters ().support_size = support_size;
  
  pcl::PointCloud<int> keypoint_indices;// 关键点序列
 narf_keypoint_detector.compute (keypoint_indices);// 计算narf关键点，并按照序列排序
  std::cout << "Found "<<keypoint_indices.size ()<<" key points.\n";

  // ----------------------------------------------
  // -----在深度图窗口显示出关键点-----
  // ----------------------------------------------
  //for (std::size_t i=0; i<keypoint_indices.size (); ++i)
   //range_image_widget.markPoint (keypoint_indices[i]%range_image.width,
                                  //keypoint_indices[i]/range_image.width);
  
  // -------------------------------------
  // -----Show keypoints in 3D viewer-----
  // -------------------------------------
  pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints_ptr (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>& keypoints = *keypoints_ptr;
  keypoints.resize (keypoint_indices.size ());
  for (std::size_t i=0; i<keypoint_indices.size (); ++i)
    keypoints[i].getVector3fMap () = range_image[keypoint_indices[i]].getVector3fMap ();
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> keypoints_color_handler (keypoints_ptr, 0, 255, 0);
  viewer.addPointCloud<pcl::PointXYZ> (keypoints_ptr, keypoints_color_handler, "keypoints");
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "keypoints");
 
  // ------------------------------------------------------
  // -----为这些关键点计算NARF描述子-------
  // ------------------------------------------------------
  std::vector<int> keypoint_indices2;
  keypoint_indices2.resize (keypoint_indices.size ());
  for (unsigned int i=0; i<keypoint_indices.size (); ++i) // This step is necessary to get the right vector type
    keypoint_indices2[i]=keypoint_indices[i];
  pcl::NarfDescriptor narf_descriptor (&range_image, &keypoint_indices2);
  narf_descriptor.getParameters ().support_size = support_size;
  narf_descriptor.getParameters ().rotation_invariant = rotation_invariant;
 pcl::PointCloud<pcl::Narf36> narf_descriptors;
 narf_descriptor.compute (narf_descriptors);
  std::cout << "Extracted "<<narf_descriptors.size ()<<" descriptors for "
                      <<keypoint_indices.size ()<< " keypoints.\n";
 
  //--------------------
  // -----Main loop-----
  //--------------------
  while (!viewer.wasStopped ())
  {
    range_image_widget.spinOnce ();  // process GUI events
    viewer.spinOnce ();
    pcl_sleep(0.01);
  }
}
