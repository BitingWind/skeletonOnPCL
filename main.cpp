#include<iostream>

#include<pcl/io/io.h>
#include<pcl/io/ply_io.h>
#include<pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include<pcl/visualization/point_cloud_color_handlers.h>

#include "SkeletonAlgorithm.h"
using namespace std;

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
	viewer->addCoordinateSystem(1.0);
	viewer->initCameraParameters();
	
	// for a plane 
	pcl::ModelCoefficients coeffs;
	for(int i = 0; i < sa.min_x; i++)
	{
		coeffs.values.push_back(1.0);// plane : a*x + b*y + c*z + d= 0;
		coeffs.values.push_back(0.0);
		coeffs.values.push_back(0.0);
		coeffs.values.push_back(-(sa.min_x + sa.DEFAULT_RADIUS * i));
		viewer->addPlane(coeffs,"plane");

		coeffs.values.clear();
	}
	
	

	std::cout<<sa.number<<" "<<sa.sample_number<<" "<<"ratio : "<<(double)sa.sample_number/sa.number;

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