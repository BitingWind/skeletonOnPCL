#pragma once

#include<pcl/point_types.h>
#include<pcl/io/io.h>
#include<pcl/io/ply_io.h>
#include<pcl/io/pcd_io.h>
#include<math.h>
#include<vector>

#include "VertexOnPCL.h"

class Coordiate
{
	int x;
	int y;
	int z;
public:
	Coordiate(void);
	Coordiate(int xx,int yy,int zz):x(xx),y(yy),z(zz)
	{

	}

	bool operator==(Coordiate other)
	{
		return x==other.x && y==other.y && z==other.z;
	}
};

class SkeletonAlgorithm
{
public:
	int number;
	int sample_number;
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_original;
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_sample;
	VertexOnPCL *vertexs;

	SkeletonAlgorithm(void);
	~SkeletonAlgorithm(void);

	//read the point data from ".ply" or ".pcd"
	void read_pcd_file(std::string filename);
	void read_ply_file(std::string filename);

	//count the static parameters !!
	void init_parameters();

	//sample points from original points
	void sample_points();
	void sample_points(unsigned int num);
	void sample_points(float ratio);

	//default radius ,respect for 10 points in the neighbour with the default radius 
    double DEFAULT_RADIUS;

	// for function grid() more than once
	float min_x,min_y,min_z,max_x,max_y,max_z;
	
	void search_neighbours(float radius);
	
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr points2cloud(std::vector<pcl::PointXYZRGBA> &pointsVec);
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr vertexs2cloud(VertexOnPCL vertexs[],int size);
	
private:
	
	void write_PLY(std::string filename,std::vector<int> point_indexs);

	// count the Axis Range( (min,max) >< (x,y,z) )and default radius(RADIUS_DEFAULT)
	void count_default_radius();

	float distance2(pcl::PointXYZRGBA &p1,pcl::PointXYZRGBA &p2);
	//make the grids via radius 
	//void grid(float radius);
};

