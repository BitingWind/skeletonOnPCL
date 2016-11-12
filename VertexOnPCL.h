#pragma once
#include<pcl/point_types.h>

class VertexOnPCL
{
public:
	unsigned int sample_index;
//	pcl::PointXYZRGBA point_data;
	
	//init false
	bool isSkeleton;
	std::vector<int> original_neighs;
	std::vector<int> sample_neighs;

	
//    VertexOnPCL(pcl::PointXYZRGBA p,unsigned int index);
	VertexOnPCL(void);
	~VertexOnPCL(void);
};

