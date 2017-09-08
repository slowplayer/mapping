#include <iostream>
#include <fstream>
using namespace std;

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/video.hpp>

#include <octomap/ColorOcTree.h>
#include <octomap/octomap.h>

#include <Eigen/Geometry>


int main(int argc,char** argv)
{
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
  
  
  
  octomap::ColorOcTree tree(0.05);
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
    
   // octomap::Pointcloud cloud;
    
    for(int v=0;v<color.rows;v++)
      for(int u=0;u<color.cols;u++)
      {
	unsigned  int d=depth.ptr<unsigned short>(v)[u];
	if(d==0)continue;
	if(d>=7000)continue;
	Eigen::Vector3d point;
	point[2]=double(d)/depthScale;
	point[0]=(u-cx)*point[2]/fx;
	point[1]=(v-cy)*point[2]/fy;
	Eigen::Vector3d pointWorld=T*point;
	
	uint8_t b,g,r;
	b=color.data[v*color.step+u*color.channels()];
	g=color.data[v*color.step+u*color.channels()+1];
	r=color.data[v*color.step+u*color.channels()+2];
	
	tree.updateNode(octomap::point3d(pointWorld[0],pointWorld[1],pointWorld[2]),true);
	tree.averageNodeColor(pointWorld[0],pointWorld[1],pointWorld[2],r,g,b);
      }
  }
  
  rgb_txt.close();
  depth_txt.close();
  gt_txt.close();
  
  
  tree.updateInnerOccupancy();
  cout<<"saving octomap ... "<<endl;
  tree.write("coloroctreenew.ot");

  return 0;
}