#include <iostream>
#include <fstream>
using namespace std;

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/video.hpp>

#include <pcl/point_types.h> 
#include <pcl/io/pcd_io.h> 
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/statistical_outlier_removal.h>


#include <Eigen/Geometry>


int main(int argc,char** argv)
{
  typedef pcl::PointXYZRGB PointT;
  typedef pcl::PointCloud<PointT> PointCloud;
  
  
  
  
  double fx,fy,cx,cy;
  fx=520.9;
  fy=521.0;
  cx=325.1;
  cy=249.7;
  double depthScale=1000;
    
  
  string rgb_path="/home/huo/Documents/mapping/data/sphere/rgb.txt";
  string depth_path="/home/huo/Documents/mapping/data/sphere/depth.txt";
  string gt_path="/home/huo/Documents/mapping/data/sphere/groundtruth.txt";
 
  ifstream rgb_txt,depth_txt,gt_txt;
  string rgb_str,depth_str;
  Eigen::Isometry3d pose;
  Eigen::Quaterniond q;
  double data[8];
  rgb_txt.open(rgb_path.c_str());
  if(!rgb_txt.is_open())
  {
    cout<<"rgb error"<<endl;
    return -1;
  }
  rgb_txt>>rgb_str;rgb_txt>>rgb_str;rgb_txt>>rgb_str;
  depth_txt.open(depth_path.c_str());
  if(!depth_txt.is_open())
  {
    cout<<"depth error"<<endl;
    return -1;
  }
  depth_txt>>depth_str;depth_txt>>depth_str;depth_txt>>depth_str;
  gt_txt.open(gt_path.c_str());
  if(!gt_txt.is_open())
  {
    cout<<"gt error"<<endl;
    return -1;
  }
  gt_txt>>rgb_str;gt_txt>>rgb_str;gt_txt>>rgb_str;
 
  PointCloud::Ptr pointCloud(new PointCloud);
  //octomap::OcTree tree(0.05);
  for(int i=0;i<200;i++)
  {
    cout<<"Image "<<i<<endl;
    if(rgb_txt.eof()||depth_txt.eof()||gt_txt.eof())
      break;
    rgb_txt>>rgb_str;rgb_txt>>rgb_str;
    depth_txt>>depth_str;depth_txt>>depth_str;
    
    
    for(int i=0;i<8;i++)
	gt_txt>>data[i];
    q=Eigen::Quaterniond(data[7],data[4],data[5],data[6]);
    pose=Eigen::Isometry3d(q);
    pose.pretranslate(Eigen::Vector3d(data[1],data[2],data[3]));
    cv::Mat color=cv::imread("/home/huo/Documents/mapping/data/sphere/"+rgb_str);
    cv::Mat depth=cv::imread("/home/huo/Documents/mapping/data/sphere/"+depth_str,-1);
    Eigen::Isometry3d T=pose;
    
    PointCloud::Ptr current(new PointCloud);
    //octomap::Pointcloud cloud;
    
    double t=(double)cv::getTickCount();
    
    for(int v=0;v<color.rows;v++)
      for(int u=0;u<color.cols;u++)
      {
	//if(i==86)cout<<u<<" "<<v<<" ";
	unsigned  int d=depth.ptr<unsigned short>(v)[u];
	//cout<<"depth:"<<d<<endl;
	//if(i==86)cout<<d<<endl;
	if(d==0)continue;
	if(d>=7000)continue;
	Eigen::Vector3d point;
	point[2]=double(d)/depthScale;
	point[0]=(u-cx)*point[2]/fx;
	point[1]=(v-cy)*point[2]/fy;
	Eigen::Vector3d pointWorld=T*point;
	
	PointT p;
	p.x=pointWorld[0];
	p.y=pointWorld[1];
	p.z=pointWorld[2];
	p.b=color.data[v*color.step+u*color.channels()];
	p.g=color.data[v*color.step+u*color.channels()+1];
	p.r=color.data[v*color.step+u*color.channels()+2];
	
	
	current->push_back(p);
//	cloud.push_back(pointWorld[0],pointWorld[1],pointWorld[2]);
      }
      
    (*pointCloud)+=*current;
    // tree.insertPointCloud(cloud,octomap::point3d(T(0,3),T(1,3),T(2,3)));
     t=(double)cv::getTickCount()-t;
     cout<<"Image "<<i<<": "<<1000.0*t/cv::getTickFrequency()<<endl;
  }
  
  rgb_txt.close();
  depth_txt.close();
  gt_txt.close();
  
  
 // tree.updateInnerOccupancy();
  cout<<"saving pointcloud ... "<<endl;
 // tree.writeBinary("octomap.bt");
  pcl::io::savePCDFileBinary("pointcloud.pcd",*pointCloud);
  return 0;
}