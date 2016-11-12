#include "SkeletonAlgorithm.h"


SkeletonAlgorithm::SkeletonAlgorithm(void)
{
	cloud_original = pcl::PointCloud<pcl::PointXYZRGBA>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBA>);
	cloud_sample = pcl::PointCloud<pcl::PointXYZRGBA>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBA>);
}


SkeletonAlgorithm::~SkeletonAlgorithm(void)
{
}

void SkeletonAlgorithm::read_pcd_file(std::string filename)
{
	pcl::PCLPointCloud2 cloud_blob;
	pcl::io::loadPCDFile (filename, cloud_blob);
	pcl::fromPCLPointCloud2 (cloud_blob, *cloud_original);
	for(int i = 0; i < cloud_original->points.size(); i++)
	{
		cloud_original->points[i].rgba = 0x2E8B57; //SeaGreen color
	}
}

void SkeletonAlgorithm::read_ply_file(std::string filename)
{
	pcl::PCLPointCloud2 cloud_blob;
	pcl::io::loadPLYFile (filename, cloud_blob);
	pcl::fromPCLPointCloud2 (cloud_blob, *cloud_original);

	for(int i = 0; i < cloud_original->points.size(); i++)
	{
		cloud_original->points[i].rgba = 0x2E8B57; //SeaGreen color
	}

}

void SkeletonAlgorithm::init_parameters()
{
	number = cloud_original->points.size();

	// count the Axis Range( (min,max) >< (x,y,z) )and default radius(RADIUS_DEFAULT)
	count_default_radius();
}

void SkeletonAlgorithm::count_default_radius()
{
	//make a alias
	auto points = cloud_original->points;

	if(number == 0)
	{
		std::cout << "The Number of Points is 0 !!! "<< std::endl;
		exit(0);
	}
	//float min_x,min_y,min_z,max_x,max_y,max_z;
	min_x = min_y = min_z =  2^10;
	max_x = max_y = max_z = - 2^10;

	

	for(int i = 0; i < number; i++)
	{
		if(points[i].x < min_x) min_x = points[i].x;
		if(points[i].x > max_x) max_x = points[i].x;
		if(points[i].y < min_y) min_y = points[i].y;
		if(points[i].y > max_y) max_y = points[i].y;
		if(points[i].z < min_z) min_z = points[i].z;
		if(points[i].z > max_z) max_z = points[i].z;
	}

	//default radius ,respect for 20 points in the neighbour with the default radius 
    DEFAULT_RADIUS = (max_x - min_x + max_y - min_y + max_z - min_z ) / number * 20 / 3;
	
	//std::cout << "RADIUS_DEFAULT £º" << RADIUS_DEFAULT << std::endl;

}

void SkeletonAlgorithm::sample_points()
{
	sample_points((unsigned int)(number * 0.06));
}

void SkeletonAlgorithm::sample_points(float ratio)
{
	sample_points((unsigned int)(number * ratio));
}

void SkeletonAlgorithm::sample_points(unsigned int num)
{
	//make a alias
	auto points = cloud_original->points;

	if(num > number)
	{
		std::cout << "The number you give is larger than size of points !!!" << std::endl;
		exit(0);
	}

	//memory the number of sampled points
	sample_number = num;

	cloud_sample->width = sample_number;
	cloud_sample->height = 1;
	cloud_sample->resize(sample_number);
	vertexs = new VertexOnPCL[sample_number]();

    bool *is_sampled = new bool[number]();
	srand(time(NULL));
	
	for(int i = 0; i < sample_number; i++)
	{
		int sample_index = rand() % number;
		if(is_sampled[sample_index])
		{
			i--; continue;
		}
		is_sampled[sample_index] = true;
		vertexs[i].sample_index = i;

		//shallow copy
		cloud_sample->points[i] = pcl::PointXYZRGBA(points[sample_index]);

		cloud_sample->points[i].rgba = 0xFF6347;  //Tomato color
	//	cloud_sample->points[i].rgba = 0xFF0000;  //red color

	}
	delete is_sampled;
}

/*
//Core algorithm : seach neighbour points via distance !!! 
//@parameter radius ; specify distance
void SkeletonAlgorithm::search_neighbours(float radius)
{
	//make a alias
	auto points = cloud->points;

	//key: grid_x && grid_y && grid_z   value: a vector that store the points' index in the specify grid
	std::map<Coordiate,std::vector<int>> Map;
	
	//grid ,make the double directory index 
	for(int i = 0; i < number; i++)
	{
	    int grid_x = (int)((points[i].x - min_x)/radius);
		int grid_y = (int)((points[i].y - min_y)/radius);
		int grid_z = (int)((points[i].z - min_z)/radius);

		Coordiate point_in_grid(grid_x,grid_y,grid_z);

		//search the key
		if(!Map.count(point_in_grid))
		{
			std::vector<int> vec;
			vec.push_back(i);
			Map[point_in_grid] = vec;
		}
		else Map[point_in_grid].push_back(i);
	}

	// store the original neighbour,search by the grid_index of sample point
	for(int i = 0; i < sample_number; i++)
	{
		int grid_x = (int)((vertexs[i].point_data.x - min_x)/radius);
		int grid_y = (int)((vertexs[i].point_data.y - min_y)/radius);
		int grid_z = (int)((vertexs[i].point_data.z - min_z)/radius);
		
		//construct a key
		Coordiate point_in_grid(grid_x,grid_y,grid_z);

		//search the key
		if(Map.count(point_in_grid))
			// shallow copy
			vertexs[i].original_neighs = std::vector<int>(Map[point_in_grid]);

		//add the points' index that it located in neighbour 26 (3*3*3 - 1) grids
		for(int j = -1; j < 2; j++)
			for(int k = -1; k < 2; k++)
				for(int l = -1; l < 2; l++)
				{
					if(j==0 && k==0 && l==0) continue;
					Coordiate point_in_grid(grid_x + j,grid_y + k,grid_z + l);
					if(Map.count(point_in_grid))
					{
						std::vector<int> search_result = Map[point_in_grid];
						for(int m = 0; m < search_result.size(); m++)
						{
							if(distance2(points[search_result[m]],vertexs[i].point_data) < radius * radius )
                                vertexs[i].original_neighs.push_back(search_result[m]);
						}
					}
						
				}
	}


}

float SkeletonAlgorithm::distance2(pcl::PointXYZRGBA &p1,pcl::PointXYZRGBA &p2)
{
	return (p1.x - p2.x) * (p1.x - p2.x) + 
		   (p1.y - p2.y) * (p1.y - p2.y) + 
		   (p1.z - p2.z) * (p1.z - p2.z);
}

*/
// a failure 
/*pcl::PointCloud<pcl::PointXYZRGBA>::Ptr SkeletonAlgorithm::points2cloud(std::vector<pcl::PointXYZRGBA> &pointsVec)
{
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
	cloud->width = 1000;
	cloud->height = 1;
	cloud->resize(1000);
//	cloud->points.resize(pointsVec.size()); 
	for(int i = 0; i < pointsVec.size(); i++)
	{
		//cloud->points[i] = pcl::PointXYZRGBA(pointsVec[i]);
		cloud->points[i].x = pointsVec[i].x;
		cloud->points[i].y = pointsVec[i].y;
		cloud->points[i].z = pointsVec[i].z;
	}
	return cloud;
}

pcl::PointCloud<pcl::PointXYZRGBA>::Ptr SkeletonAlgorithm::vertexs2cloud(VertexOnPCL vertexs[],int size)
{
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
//	cloud->points.resize(size); 
	for(int i = 0; i < size; i++)
		cloud->points[i] = pcl::PointXYZRGBA(vertexs[i].point_data);
	return cloud;
}*/