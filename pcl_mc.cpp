/**
 *
 * @file      pcl_mc.cpp
 * @brief     
 * @details   
 * 
 * @author    Lars Kiesow (lkiesow), lkiesow@uos.de, Universität Osnabrück
 * @version   
 * @date      02/28/2012 02:59:44 PM
 *
 **/

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/marching_cubes.h>
#include <pcl/surface/marching_cubes_greedy.h>
#include <pcl/features/normal_3d.h>
#include <pcl/search/search.h>
#include <iostream>
#include <cmath>


//typedef pcl::PointXYZ PointType; 
typedef pcl::PointXYZRGB PointType; 
typedef pcl::Normal Normals;	
//typedef pcl::PointNormal PointTypeNormal; 
typedef pcl::PointXYZRGBNormal PointTypeNormal; 


template<typename T>
class PCLMarchingCubesGreedyWrapper : public pcl::MarchingCubesGreedy<T>
{
	public:
		PCLMarchingCubesGreedyWrapper() {};
		virtual ~PCLMarchingCubesGreedyWrapper() {};

	protected:
		virtual void performReconstruction(pcl::PointCloud<T    >&, std::vector<pcl::Vertices>&)
		{
			std::cout << "IS THIS EVER USED?" << std::endl;
		}
};


int main (int argc, char** argv) {

	if ( argc != 5 ) {
		std::cout << "Usage: " << argv[0] << " leafSize isoLevel infile.pcd outfile.ply" << std::endl;
		return 0;
	}

	double leafSize = atof(argv[1]);  // 0.01
	float isoLevel  = atof(argv[2]);  // 0.5

	// Load input file */
	pcl::PointCloud<PointType>::Ptr pointcloud (new pcl::PointCloud<PointType> ());
	pcl::PointCloud<PointTypeNormal>::Ptr pointcloudNormal (new pcl::PointCloud<PointTypeNormal> ());
	pcl::io::loadPCDFile ( argv[3], *pointcloud);

	/* Marching Cubes Reconstruction */
	pcl::PolygonMesh mesh;

	// Normal estimation*
	pcl::NormalEstimation<PointType, PointTypeNormal> norm_est;
	pcl::PointCloud<Normals>::Ptr normals (new pcl::PointCloud<Normals>);
	pcl::search::KdTree<PointType>::Ptr tree (new pcl::search::KdTree<PointType>);
	tree->setInputCloud (pointcloud);
	norm_est.setInputCloud (pointcloud);
	norm_est.setSearchMethod (tree);
	norm_est.setKSearch(30);
	norm_est.compute(*pointcloudNormal);
	pcl::copyPointCloud (*pointcloud, *pointcloudNormal);
	pointcloud.reset();

	// Create the search method
	pcl::search::KdTree<PointTypeNormal>::Ptr tree2 (new pcl::search::KdTree<PointTypeNormal>);
	tree2->setInputCloud (pointcloudNormal);
	// Initialize objects
	PCLMarchingCubesGreedyWrapper<PointTypeNormal> mc;
	// Set parameters
	mc.setLeafSize(leafSize);  
	mc.setIsoLevel(isoLevel);   //ISO: must be between 0 and 1.0
	mc.setSearchMethod(tree2);
	mc.setInputCloud(pointcloudNormal);
	// Reconstruct
	mc.reconstruct (mesh);

	//Saving to disk in VTK format:
	//pcl::io::saveVTKFile (argv[2], mesh);
	pcl::io::savePLYFile (argv[4], mesh);
}
