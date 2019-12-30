#pragma once
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/voxel_grid.h>

#include <iostream>
#include <vector>
#include <string>
#include<cmath>
#include <thread>
#include <fstream>
#include <algorithm>
#include <time.h>
#include <vector>


using namespace std;
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;


class AreaPick
{
public:
	AreaPick();
	~AreaPick(){}
	virtual void loadInputcloud(string inputcloudfile);
	virtual void stl_ply(string stl_path,string ply_path);
	virtual void simpleViewer(string inputcloudfile);
	PointCloudT::Ptr get_picked_area();
	PointCloudT::Ptr get_input_cloud();
protected:
	virtual void closeviewer(const pcl::visualization::KeyboardEvent& event, void* args);
	virtual void pp_callback(const pcl::visualization::AreaPickingEvent& event, void* args);
private:
	PointCloudT::Ptr inputcloud;
	PointCloudT::Ptr cloud_filtered;
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
	PointCloudT::Ptr clicked_points_3d;
	int num ;
	string cloudName;
};


class Pointspick:public AreaPick
{
public:
	Pointspick();
	~Pointspick(){}
	void loadInputcloud(string inputcloudfile);
	void stl_ply(string stl_path,string ply_path);
	void simpleViewer(const string inputcloudfile);
	int find_nexttolast(const string filepath);
	PointCloudT::Ptr get_picked_points();
protected:
	void pp_callback(const pcl::visualization::PointPickingEvent& event, void*);
private:
	PointCloudT::Ptr inputcloud;
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
	pcl::PointCloud<pcl::PointXYZ>::Ptr clicked_points_3d;
	string cloudName;
	int num;
};
