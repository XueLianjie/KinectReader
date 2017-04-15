#include <stdlib.h>  
#include <iostream>  
#include <string>  
//OpenNI  
#include <XnCppWrapper.h>  //OpenNI c++ 头文件
//OpenCV
#include "opencv/cv.h"  
#include "opencv/highgui.h"  
//PCL
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/people/ground_based_people_detection_app.h>//这都有？？


#include "kinectreader.h"
//#include "config.h"   
boost::shared_ptr<pcl::visualization::PCLVisualizer> simpleVis (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud_no_plane,pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud_plane);

int main( int argc, char** argv )  
{  
  
  boost::shared_ptr<pcl::visualization::PCLVisualizer> pclvisualizer(new pcl::visualization::PCLVisualizer ("3D Viewer"));
  //pcl::visualization::CloudViewer viewer("cloud");//点云显示
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr pPointCloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
  pcl::ExtractIndices<pcl::PointXYZ> extract;//点提取对象**需要包含#include<pcl/filters/extract_indices.h>
  pcl::ExtractIndices<pcl::Normal> extract_normals;//点提取对象
  
  
  pcl::ModelCoefficients::Ptr coefficients_plane (new pcl::ModelCoefficients); //coefficients_cylinder(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers_plane(new pcl::PointIndices);
  
  
  char key=0,times=0; 
  Kinectreader reader;
  reader.Init();
  while(key != 27)
  {
    reader.ReadKinect();
    reader.ShowDepthAndImg();
    pPointCloud = reader.ToPointCloud();	    
    
    //点云分割
    /*
     * 先计算点云的法向量
     * 然后进行分割，获得内点索引，和拟合模型参数
     * 提取模型内点到新的点云对象中
     * 创建投影对象，将去除平面的点云向平面上投影，获得投影后的点云对象
     */
    
    ne.setSearchMethod(tree);
    ne.setInputCloud(pPointCloud);
    ne.setKSearch(40);
    ne.compute(*cloud_normals);
    
    //设置分割所用的模型类型、方法和相关参数
    seg.setOptimizeCoefficients(true);//设置为false 后点云出现略微抖动，不太稳定
    seg.setModelType(pcl::SACMODEL_NORMAL_PLANE);
    seg.setNormalDistanceWeight(0.08);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(100);
    seg.setDistanceThreshold(0.06);
    seg.setInputCloud(pPointCloud);
    seg.setInputNormals(cloud_normals);
    //执行分割，获取模型参数和内点
    seg.segment(*inliers_plane, *coefficients_plane);
    std::cerr << "Plane coefficients: " << *coefficients_plane << std::endl;
    extract.setInputCloud(pPointCloud);
    extract.setIndices(inliers_plane);
    extract.setNegative(true);//是否显示物体
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_no_plane(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>());
    extract.filter(*cloud_no_plane);
    extract.setNegative(false);//是否显示物体
    extract.filter(*cloud_plane);
    //projection
    pcl::PointCloud<pcl::PointXYZ>::Ptr projectedPoints(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::ProjectInliers<pcl::PointXYZ> projection;
    projection.setModelType(pcl::SACMODEL_PLANE);
    projection.setInputCloud(cloud_no_plane);
    
    projection.setModelCoefficients(coefficients_plane);
    
    projection.filter(*projectedPoints);
    //viewer.showCloud(pPointCloud); 
    /*
     * updatePointCloud函数定义
     * 

template<typename PointT >
bool pcl::visualization::PCLVisualizer::updatePointCloud 	( 	const typename pcl::PointCloud< PointT >::ConstPtr &  	cloud,
		const PointCloudColorHandler< PointT > &  	color_handler,
		const std::string &  	id = "cloud" 
	) 		

Updates the XYZ data for an existing cloud object id on screen.

Parameters
    [in]	cloud	the input point cloud dataset
    [in]	color_handler	the color handler to use
    [in]	id	the point cloud object id to update (default: cloud) 
    */
    if(times == 0)//只添加一次点云，以后只需要更新即可
    {
      times = 1;
      pclvisualizer = simpleVis(cloud_no_plane, cloud_plane);
    }
    
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color_green(cloud_no_plane, 0, 255, 0);
    pclvisualizer->updatePointCloud<pcl::PointXYZ> (cloud_no_plane, single_color_green, "no_plane");//自动移除id为“sample cloud”的点云，并添加新的点云
    //pclvisualizer->spinOnce (100);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color_red(cloud_plane, 255, 0, 0);
    pclvisualizer->updatePointCloud<pcl::PointXYZ> (cloud_plane, single_color_red, "plane");
    pclvisualizer->spinOnce(100);//这一句必须得加，以使得视窗可以读取数据和显示
    key=cvWaitKey(20);//必须得加上此句，来让深度图和彩图顺利显示
  }
  reader.StopRead();
  
  return 0; 
}

//此函数只调用一次
boost::shared_ptr<pcl::visualization::PCLVisualizer> simpleVis (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud_no_plane, pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud_plane )//返回指向视窗的指针的函数，参数为点云常指针
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));//创建一个视窗对象指针并赋值
  viewer->setBackgroundColor (0, 0, 0);//背景颜色设置为黑色
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color_green(cloud_no_plane, 0, 255, 0);//设置地面上物体点云为纯绿色
  viewer->addPointCloud<pcl::PointXYZ> (cloud_no_plane, single_color_green, "no_plane");//添加绿色点云并设置ID号为“no_plane”
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color_red(cloud_plane, 255, 0, 0);//红色地面点云
  viewer->addPointCloud<pcl::PointXYZ> (cloud_plane, single_color_red, "plane");//添加红色地面点云
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "no_plane");//地面上物体点云设置大小为3
  viewer->addCoordinateSystem (1.0);//添加坐标系
  viewer->initCameraParameters ();//Initialize camera parameters with some default values. 
  return (viewer);//返回智能指针对象
}