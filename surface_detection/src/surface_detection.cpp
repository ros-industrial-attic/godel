/*
 * surface_detection.cpp
 *
 *  Created on: Feb 11, 2014
 *      Author: ros-industrial
 */

#include <surface_detection/surface_detection.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/filters/voxel_grid.h>

namespace surface_detection {

SurfaceDetection::SurfaceDetection():
		acquired_clouds_counter_(0),
		k_search_(defaults::K_SEARCH),
		meanK_(defaults::STATISTICAL_OUTLIER_MEAN),
		stdv_threshold_(defaults::STATISTICAL_OUTLIER_STDEV_THRESHOLD),
		rg_min_cluster_size_(defaults::REGION_GROWING_MIN_CLUSTER_SIZE),
		rg_max_cluster_size_(defaults::REGION_GROWING_MAX_CLUSTER_SIZE),
		rg_neightbors_(defaults::REGION_GROWING_NEIGHBORS),
		rg_smoothness_threshold_(defaults::REGION_GROWING_SMOOTHNESS_THRESHOLD),
		rg_curvature_threshold_(defaults::REGION_GROWING_CURVATURE_THRESHOLD),
		acquisition_time_(defaults::ACQUISITION_TIME),
		tr_search_radius_(defaults::TRIANGULATION_SEARCH_RADIUS),
		tr_mu_(defaults::TRIANGULATION_MU),
		tr_max_nearest_neighbors_(defaults::TRIANGULATION_MAX_NEAREST_NEIGHBORS),
		tr_max_surface_angle_(defaults::TRIANGULATION_MAX_SURFACE_ANGLE),
		tr_min_angle_(defaults::TRIANGULATION_MIN_ANGLE),
		tr_max_angle_(defaults::TRIANGULATION_MAX_ANGLE),
		tr_normal_consistency_(defaults::TRIANGULATION_NORMAL_CONSISTENCY),
		voxel_leafsize_(defaults::VOXEL_LEAF_SIZE)
{
	// TODO Auto-generated constructor stub

}

SurfaceDetection::~SurfaceDetection()
{
	// TODO Auto-generated destructor stub
	ROS_INFO_STREAM("Surface Detection destructor invoked");
}

void SurfaceDetection::set_acquisition_time(float val)
{
	acquisition_time_ = val;
}

Cloud::Ptr SurfaceDetection::get_acquired_cloud()
{
	return acquired_cloud_ptr_;
}

CloudRGB::Ptr SurfaceDetection::get_region_colored_cloud()
{
	return region_colored_cloud_ptr_;
}

std::vector<Cloud::Ptr> SurfaceDetection::get_segment_clouds()
{
	return segment_clouds_;
}

bool SurfaceDetection::acquire_data()
{
	// initialize  aggregate point cloud
	acquired_cloud_ptr_.reset(new Cloud());
	acquired_clouds_counter_ = 0;

	// initialize subscriber
	ros::NodeHandle nh;
	point_cloud_subs_ = nh.subscribe(config::POINT_COLUD_TOPIC,1,
			&SurfaceDetection::point_cloud_subscriber_cb,this);

	// wait for topic
	sensor_msgs::PointCloud2ConstPtr msg =
			ros::topic::waitForMessage<sensor_msgs::PointCloud2>(point_cloud_subs_.getTopic(),
					ros::Duration(5.0f));

	if(msg)
	{
		ROS_INFO_STREAM("Started point cloud acquisition");

		ros::Duration acquistion_time(acquisition_time_);
		ros::Time start_time = ros::Time::now();
		while(ros::ok() &&
				((start_time + acquistion_time) > ros::Time::now()))
		{
			ros::spinOnce();
		}

		point_cloud_subs_.shutdown();
		ROS_INFO_STREAM("Finished point cloud acquisition");

		return !acquired_cloud_ptr_->empty();
	}
	else
	{
		ROS_ERROR_STREAM("Point cloud topic: '"<<point_cloud_subs_.getTopic()<<
				"' not advertised");
		return false;
	}
}

bool SurfaceDetection::find_surfaces()
{
	// reset members
	region_colored_cloud_ptr_ = CloudRGB::Ptr(new CloudRGB());
	segment_clouds_.clear();

	// variables to hold intermediate results
	Normals::Ptr normals(new Normals());
	std::vector<pcl::PointIndices> clusters_indices;
	std::vector<Normals::Ptr> segment_normals;

	if(apply_voxel_downsampling(*acquired_cloud_ptr_))
	{
		ROS_INFO_STREAM("Voxel downsampling succeeded, downsampled cloud size: "<<
				acquired_cloud_ptr_->size());
	}

	// apply statistical filter
	if(apply_statistical_filter(*acquired_cloud_ptr_,*acquired_cloud_ptr_))
	{
		ROS_INFO_STREAM("Statistical filter succeeded");
	}
	else
	{
		ROS_ERROR_STREAM("Statistical filter failed");
		return false;
	}

	// estimate normals
	if(apply_normal_estimation(*acquired_cloud_ptr_,*normals))
	{
		ROS_INFO_STREAM("Normal estimation succeeded");
	}
	else
	{
		ROS_ERROR_STREAM("Normal estimation failed");
		return false;
	}

	// applying region growing segmentation
	if(apply_region_growing_segmentation(*acquired_cloud_ptr_,*normals,clusters_indices,
			*region_colored_cloud_ptr_))
	{
		ROS_INFO_STREAM("Region growing succeeded: "<<
				clusters_indices.size()<< " cluster found");

		// filling cloud array
		for(int i =0;i< clusters_indices.size(); i++)
		{
			Cloud::Ptr segment_cloud_ptr(new Cloud());
			Normals::Ptr segment_normal_ptr(new Normals());
			pcl::copyPointCloud(*acquired_cloud_ptr_,clusters_indices[i],
					*segment_cloud_ptr);
			pcl::copyPointCloud(*normals,clusters_indices[i],*segment_normal_ptr);
			segment_clouds_.push_back(segment_cloud_ptr);
			segment_normals.push_back(segment_normal_ptr);
		}
	}
	else
	{
		ROS_ERROR_STREAM("Region growing failed");
		return false;
	}

	return true;
}

void SurfaceDetection::point_cloud_subscriber_cb(const sensor_msgs::PointCloud2ConstPtr &msg)
{
	// convert to message to point cloud
	Cloud::Ptr new_cloud_ptr(new Cloud());
	pcl::fromROSMsg<pcl::PointXYZ>(*msg,*new_cloud_ptr);

	std::vector<int> index;
	pcl::removeNaNFromPointCloud(*new_cloud_ptr,*new_cloud_ptr,index);

	// add to aggregate point cloud
	(*acquired_cloud_ptr_)+=*new_cloud_ptr;
	acquired_clouds_counter_++;

	ROS_INFO_STREAM("Added " <<acquired_clouds_counter_ <<
			" point clouds, total points: "<<acquired_cloud_ptr_->points.size());
}

bool SurfaceDetection::apply_statistical_filter(const Cloud& in,Cloud& out)
{
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> filter;
	Cloud::ConstPtr cloud_ptr = boost::make_shared<Cloud>(in);
	filter.setInputCloud(cloud_ptr);
	filter.setMeanK(meanK_);
	filter.setStddevMulThresh(stdv_threshold_);
	filter.filter(out);

	return !out.empty();
}

bool SurfaceDetection::apply_normal_estimation(const Cloud &cloud,Normals& normals)
{
	Cloud::ConstPtr cloud_ptr = boost::make_shared<Cloud>(cloud);
	pcl::search::Search<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	pcl::NormalEstimation<pcl::PointXYZ,pcl::Normal> normal_estimator;
	normal_estimator.setSearchMethod(tree);
	normal_estimator.setInputCloud(cloud_ptr);
	normal_estimator.setKSearch(k_search_);
	normal_estimator.compute(normals);

	return !normals.empty();
}

bool SurfaceDetection::apply_region_growing_segmentation(const Cloud& in,
		const Normals& normals,
		std::vector<pcl::PointIndices>& clusters,
		CloudRGB& colored_cloud)
{
	Cloud::ConstPtr cloud_ptr = boost::make_shared<Cloud>(in);
	const Normals::Ptr normals_ptr = boost::make_shared<Normals>(normals);
	pcl::search::Search<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	pcl::RegionGrowing<pcl::PointXYZ,pcl::Normal> rg;
	rg.setMinClusterSize(rg_min_cluster_size_);
	rg.setMaxClusterSize(rg_max_cluster_size_);
	rg.setSearchMethod(tree);
	rg.setNumberOfNeighbours(rg_neightbors_);
	rg.setInputCloud(cloud_ptr);
	rg.setInputNormals(normals_ptr);
	rg.extract(clusters);

	pcl::copyPointCloud(*rg.getColoredCloud(),colored_cloud);


	return clusters.size()>0;
}

bool SurfaceDetection::apply_fast_triangulation(const Cloud& in,
		const Normals& normals,
		pcl::PolygonMesh& mesh)
{
	// search tree
	pcl::search::KdTree<pcl::PointNormal>::Ptr tree (
			new pcl::search::KdTree<pcl::PointNormal>);

	// cloud with normals
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(
			new pcl::PointCloud<pcl::PointNormal>());
	pcl::concatenateFields(in,normals,*cloud_with_normals);
	tree->setInputCloud(cloud_with_normals);

	// triangulation
	pcl::GreedyProjectionTriangulation<pcl::PointNormal> gt;
	gt.setSearchRadius(tr_search_radius_);
	gt.setMu(tr_mu_);
	gt.setMaximumNearestNeighbors(tr_max_nearest_neighbors_);
	gt.setMaximumSurfaceAngle(tr_max_surface_angle_);
	gt.setMinimumAngle(tr_min_angle_);
	gt.setMaximumAngle(tr_max_angle_);
	gt.setNormalConsistency(tr_normal_consistency_);

	gt.setInputCloud(cloud_with_normals);
	gt.setSearchMethod(tree);
	gt.reconstruct(mesh);

	return mesh.polygons.size() > 0;

}

bool SurfaceDetection::apply_voxel_downsampling(Cloud& cloud)
{
	// converting to pcl2 type
	 pcl::PCLPointCloud2::Ptr pcl_cloud_ptr(new pcl::PCLPointCloud2());
	 pcl::toPCLPointCloud2(cloud,*pcl_cloud_ptr);


	pcl::VoxelGrid<pcl::PCLPointCloud2> vg;
	vg.setInputCloud(pcl_cloud_ptr);
	ROS_INFO_STREAM("voxel grid setting leaf size");
	vg.setLeafSize(voxel_leafsize_,voxel_leafsize_,voxel_leafsize_);

	ROS_INFO_STREAM("voxel grid filtering");
	vg.filter(*pcl_cloud_ptr);

	pcl::fromPCLPointCloud2(*pcl_cloud_ptr,cloud);
	return true;
}


} /* namespace surface_detection */
