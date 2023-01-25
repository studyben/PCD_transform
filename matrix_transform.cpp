
#include <iostream>
 
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>	//	pcl::transformPointCloud 用到这个头文件
#include <pcl/visualization/pcl_visualizer.h>


void showHelp(char * program_name)
{
  std::cout << std::endl;
  std::cout << "Usage: " << program_name << " cloud_filename.[pcd|ply]" << std::endl;
  std::cout << "-h:  Show this help." << std::endl;
}


int
main (int argc, char** argv)
{
 
  // 如果没有输入预期的参数程序将显示帮助
  if (pcl::console::find_switch (argc, argv, "-h") || pcl::console::find_switch (argc, argv, "--help")) {
    showHelp (argv[0]);
    return 0;
  }
 
  // 从主函数参数查找点云数据文件 (.PCD|.PLY)
  std::vector<int> filenames;
  bool file_is_pcd = false;
 
  filenames = pcl::console::parse_file_extension_argument (argc, argv, ".ply");
 
  if (filenames.size () != 1)  {
    filenames = pcl::console::parse_file_extension_argument (argc, argv, ".pcd");
 
    if (filenames.size () != 1) {
      showHelp (argv[0]);
      return -1;
    } else {
      file_is_pcd = true;
    }
  }
 
  // 加载点云数据文件
  pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
 
  if (file_is_pcd) {
    if (pcl::io::loadPCDFile (argv[filenames[0]], *source_cloud) < 0)  {
      std::cout << "Error loading point cloud " << argv[filenames[0]] << std::endl << std::endl;
      showHelp (argv[0]);
      return -1;
    }
  } else {
    if (pcl::io::loadPLYFile (argv[filenames[0]], *source_cloud) < 0)  {
      std::cout << "Error loading point cloud " << argv[filenames[0]] << std::endl << std::endl;
      showHelp (argv[0]);
      return -1;
    }
  }
 
  /* 提示: 变换矩阵工作原理 :
           |-------> 变换矩阵列
    | 1 0 0 x |  \
    | 0 1 0 y |   }-> 左边是一个3阶的单位阵(无旋转)
    | 0 0 1 z |  /
    | 0 0 0 1 |    -> 这一行用不到 (这一行保持 0,0,0,1)
    方法一 #1: 使用 Matrix4f
    这个是“手工方法”，可以完美地理解，但容易出错!
  */
  Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();
 
  // 定义一个旋转矩阵 (见 https://en.wikipedia.org/wiki/Rotation_matrix)
  float theta = M_PI/40; // 弧度角
  transform_1 (0,0) = cos (theta);
  transform_1 (0,1) = -sin(theta);
  transform_1 (1,0) = sin (theta);
  transform_1 (1,1) = cos (theta);
  //    	(行, 列)
 
  // 在 X 轴上定义一个 2.5 米的平移.
  transform_1 (0,3) = 2.5;
 
  // 打印变换矩阵
  printf ("Method #1: using a Matrix4f\n");
  std::cout << transform_1 << std::endl;
 
  /*  方法二 #2: 使用 Affine3f
    这种方法简单，不易出错
  */
  Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();
 
  // 在 X 轴上定义一个 2.5 米的平移.
  transform_2.translation() << 0.0, 0.0, 0.0;
 
  // 和前面一样的旋转; Z 轴上旋转 theta 弧度
  transform_2.rotate (Eigen::AngleAxisf (theta, Eigen::Vector3f::UnitZ()));
 
  // 打印变换矩阵
  printf ("\nMethod #2: using an Affine3f\n");
  std::cout << transform_2.matrix() << std::endl;
 
  // 执行变换，并将结果保存在新创建的transformed_cloud中
  pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
  // 可以使用 transform_1 或 transform_2; t它们是一样的
  pcl::transformPointCloud (*source_cloud, *transformed_cloud, transform_2);

  pcl::io::savePCDFileASCII("a.pcd", *transformed_cloud);
  return 0;
}
