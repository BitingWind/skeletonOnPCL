#include<iostream>

#include<pcl/io/io.h>
#include<pcl/io/ply_io.h>
#include<pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include<pcl/visualization/point_cloud_color_handlers.h>

#include "SkeletonAlgorithm.h"
using namespace std;

string makeString(const char s[], int id)
{
	s = s + id;
	return s;
}
void addGrid27(SkeletonAlgorithm &sa,int index,boost::shared_ptr<pcl::visualization::PCLVisualizer> &viewer)
{
	
	int grid_x = (int)((sa.cloud_sample->points[index].x - sa.min_x)/sa.DEFAULT_RADIUS);
	int grid_y = (int)((sa.cloud_sample->points[index].y - sa.min_y)/sa.DEFAULT_RADIUS);
	int grid_z = (int)((sa.cloud_sample->points[index].z - sa.min_z)/sa.DEFAULT_RADIUS);

	float coe_x = sa.min_x + (grid_x + 1/2) * sa.DEFAULT_RADIUS;
	float coe_y = sa.min_y + (grid_y + 1/2) * sa.DEFAULT_RADIUS;
	float coe_z = sa.min_z + (grid_z + 1/2) * sa.DEFAULT_RADIUS;

	int id = 0;

	pcl::ModelCoefficients coeffs;
	for(int i = -1; i < 2; i++)
		for(int j = -1; j < 2; j++)
			for(int k = -1; k < 2; k++)
			{
				coeffs.values.push_back(coe_x + i * sa.DEFAULT_RADIUS);// Cube : x,y,z, Qx,Qy,Qz,Qw, width, height, depth
				coeffs.values.push_back(coe_y + j * sa.DEFAULT_RADIUS);
				coeffs.values.push_back(coe_z + k * sa.DEFAULT_RADIUS);
				coeffs.values.push_back(0.0);
				coeffs.values.push_back(0.0);
				coeffs.values.push_back(0.0);
				coeffs.values.push_back(0.0);
				coeffs.values.push_back(sa.DEFAULT_RADIUS);
				coeffs.values.push_back(sa.DEFAULT_RADIUS);
				coeffs.values.push_back(sa.DEFAULT_RADIUS);

				//coeffs.values.push_back(-(sa.min_x + sa.DEFAULT_RADIUS * i));
				viewer->addCube(coeffs,makeString("cube_id: ", (i+1)*100+(j+1)*10+ k+1));
				coeffs.values.clear();
			}
	

	

}
int main()
{
	SkeletonAlgorithm sa;
	sa.read_ply_file("3_Y_noisy.ply");
	sa.init_parameters();
	sa.sample_points();

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA> redColor(sa.cloud_original,255,0,0);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA> seaGreenColor(sa.cloud_sample,46,139,87);

	//viewer->setBackgroundColor(75.0/255.0,75.0/255.0,75.0/255.0); //shallow gray
	viewer->addPointCloud<pcl::PointXYZRGBA>(sa.cloud_original,redColor,"original cloud");
	viewer->addPointCloud<pcl::PointXYZRGBA>(sa.cloud_sample,seaGreenColor,"sample cloud");
	
	

	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
	viewer->addCoordinateSystem(1000);
	viewer->initCameraParameters();	
	
	addGrid27(sa,100,viewer);

	std::cout<<sa.number<<" "<<sa.sample_number<<" "<<"ratio : "<<(double)sa.sample_number/sa.number;
	std::cout<<"radius: "<<sa.DEFAULT_RADIUS;

	//pcl::visualization::CloudViewer viewer("test viewer");
	
	//viewer.showCloud(sa.vertexs2cloud(sa.vertexs,sa.sample_number));
	//viewer.showCloud(sa.cloud_sample);

	
	while(!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
	return 0;
}