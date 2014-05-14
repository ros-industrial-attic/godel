/*
	Copyright Feb 11, 2014 Southwest Research Institute

	Licensed under the Apache License, Version 2.0 (the "License");
	you may not use this file except in compliance with the License.
	You may obtain a copy of the License at

		http://www.apache.org/licenses/LICENSE-2.0

	Unless required by applicable law or agreed to in writing, software
	distributed under the License is distributed on an "AS IS" BASIS,
	WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
	See the License for the specific language governing permissions and
	limitations under the License.
*/

#include <godel_surface_detection/detection/surface_detection.h>
#include <pcl_ros/transforms.h>
#include <tf/transform_listener.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/surface/mls.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <tf/transform_datatypes.h>
#include <octomap_ros/conversions.h>

namespace godel_surface_detection { namespace detection{

SurfaceDetection::SurfaceDetection():
		acquired_clouds_counter_(0),
		octree_(new octomap::OcTree(defaults::VOXEL_LEAF_SIZE)),
		full_cloud_ptr_(new Cloud())
{

	params_.frame_id = defaults::FRAME_ID;
	params_.k_search=defaults::K_SEARCH;
	params_.meanK=defaults::STATISTICAL_OUTLIER_MEAN;
	params_.stdv_threshold=defaults::STATISTICAL_OUTLIER_STDEV_THRESHOLD;
	params_.rg_min_cluster_size=defaults::REGION_GROWING_MIN_CLUSTER_SIZE;
	params_.rg_max_cluster_size=defaults::REGION_GROWING_MAX_CLUSTER_SIZE;
	params_.rg_neightbors=defaults::REGION_GROWING_NEIGHBORS;
	params_.rg_smoothness_threshold=defaults::REGION_GROWING_SMOOTHNESS_THRESHOLD;
	params_.rg_curvature_threshold=defaults::REGION_GROWING_CURVATURE_THRESHOLD;
	params_.tr_search_radius=defaults::TRIANGULATION_SEARCH_RADIUS;
	params_.tr_mu=defaults::TRIANGULATION_MU;
	params_.tr_max_nearest_neighbors=defaults::TRIANGULATION_MAX_NEAREST_NEIGHBORS;
	params_.tr_max_surface_angle=defaults::TRIANGULATION_MAX_SURFACE_ANGLE;
	params_.tr_min_angle=defaults::TRIANGULATION_MIN_ANGLE;
	params_.tr_max_angle=defaults::TRIANGULATION_MAX_ANGLE;
	params_.tr_normal_consistency=defaults::TRIANGULATION_NORMAL_CONSISTENCY;
	params_.voxel_leafsize=defaults::VOXEL_LEAF_SIZE;
	params_.marker_alpha=defaults::MARKER_ALPHA;
	params_.ignore_largest_cluster=defaults::IGNORE_LARGEST_CLUSTER;
	params_.use_octomap=defaults::USE_OCTOMAP;
	params_.occupancy_threshold=defaults::OCCUPANCY_THRESHOLD;
	params_.mls_upsampling_radius=defaults::MLS_UPSAMPLING_RADIUS;
	params_.mls_point_density=defaults::MLS_POINT_DENSITY;
	params_.mls_search_radius=defaults::MLS_SEARCH_RADIUS;
	params_.use_tabletop_seg=defaults::USE_TABLETOP_SEGMENTATION;
	params_.tabletop_seg_distance_threshold=defaults::TABLETOP_SEG_DISTANCE_THRESH;

	srand(time(NULL));
	clear_results();
}

SurfaceDetection::~SurfaceDetection()
{
	// TODO Auto-generated destructor stub
}

bool SurfaceDetection::init()
{
	octree_->setResolution(params_.voxel_leafsize);
	octree_->setOccupancyThres(params_.occupancy_threshold);
	full_cloud_ptr_->header.frame_id = params_.frame_id;
	acquired_clouds_counter_ = 0;
	return true;
}

void SurfaceDetection::clear_results()
{
	acquired_clouds_counter_ = 0;
	full_cloud_ptr_->clear();
	surface_clouds_.clear();
	meshes_.markers.clear();
}

bool SurfaceDetection::load_parameters(std::string node_ns)
{
	return load_parameters(params_,node_ns);
}

bool SurfaceDetection::load_parameters(godel_msgs::SurfaceDetectionParameters &params,std::string node_ns)
{
	ros::NodeHandle nh(node_ns);

	// bool parameters
	bool tr_normal_consistency, use_tabletop_seg, ignore_largest_cluster;

	bool succeeded;
	if(		nh.getParam(params::FRAME_ID,params.frame_id)&&
			nh.getParam(params::K_SEARCH,params.k_search) &&
			nh.getParam(params::STOUTLIER_MEAN,params.meanK) &&
			nh.getParam(params::STOUTLIER_STDEV_THRESHOLD,params.stdv_threshold) &&
			nh.getParam(params::REGION_GROWING_MIN_CLUSTER_SIZE,params.rg_min_cluster_size) &&
			nh.getParam(params::REGION_GROWING_MAX_CLUSTER_SIZE,params.rg_max_cluster_size) &&
			nh.getParam(params::REGION_GROWING_NEIGHBORS,params.rg_neightbors) &&
			nh.getParam(params::REGION_GROWING_SMOOTHNESS_THRESHOLD,params.rg_smoothness_threshold) &&
			nh.getParam(params::REGION_GROWING_CURVATURE_THRESHOLD,params.rg_curvature_threshold) &&
			nh.getParam(params::TRIANGULATION_SEARCH_RADIUS,params.tr_search_radius) &&
			nh.getParam(params::TRIANGULATION_MU ,params.tr_mu) &&
			nh.getParam(params::TRIANGULATION_MAX_NEAREST_NEIGHBORS,params.tr_max_nearest_neighbors) &&
			nh.getParam(params::TRIANGULATION_MAX_SURFACE_ANGLE,params.tr_max_surface_angle) &&
			nh.getParam(params::TRIANGULATION_MIN_ANGLE,params.tr_min_angle) &&
			nh.getParam(params::TRIANGULATION_MAX_ANGLE,params.tr_max_angle) &&
			nh.getParam(params::TRIANGULATION_NORMAL_CONSISTENCY,tr_normal_consistency) &&
			nh.getParam(params::VOXEL_LEAF_SIZE,params.voxel_leafsize) &&
			nh.getParam(params::OCCUPANCY_THRESHOLD,params.occupancy_threshold) &&
			nh.getParam(params::MLS_UPSAMPLING_RADIUS,params.mls_upsampling_radius) &&
			nh.getParam(params::MLS_POINT_DENSITY,params.mls_point_density) &&
			nh.getParam(params::MLS_SEARCH_RADIUS,params.mls_search_radius) &&
			nh.getParam(params::USE_TABLETOP_SEGMENTATION,use_tabletop_seg) &&
			nh.getParam(params::TABLETOP_SEG_DISTANCE_THRESH,params.tabletop_seg_distance_threshold) &&
			nh.getParam(params::MARKER_ALPHA,params.marker_alpha) &&
			nh.getParam(params::IGNORE_LARGEST_CLUSTER,ignore_largest_cluster)
			)
	{
		params.tr_normal_consistency = tr_normal_consistency;
		params.use_tabletop_seg = use_tabletop_seg;
		params.ignore_largest_cluster = ignore_largest_cluster;
		succeeded = true;
		ROS_INFO_STREAM("surface detection parameters loaded");
	}
	else
	{
		succeeded = false;
		ROS_ERROR_STREAM("surface detection parameter(s) not found under namespace: "<<nh.getNamespace());
	}

	return succeeded;
}

void SurfaceDetection::mesh_to_marker(const pcl::PolygonMesh &mesh,
		visualization_msgs::Marker &marker)
{
	// color value ranges
	static const double color_val_min = 0.5f;
	static const double color_val_max = 1.0f;
	std_msgs::ColorRGBA color;
	color.a =  1;

	// set marker properties
	tf::poseTFToMsg(tf::Transform::getIdentity(),marker.pose );
	marker.scale.x = marker.scale.y = marker.scale.z = 1;
	marker.type = marker.TRIANGLE_LIST;
	marker.action = marker.ADD;

	// create color
	color.r = color_val_min +
			(static_cast<double>(rand())/static_cast<double>(RAND_MAX))
			* (color_val_max - color_val_min);
	color.g = color_val_min +
					(static_cast<double>(rand())/static_cast<double>(RAND_MAX))
					* (color_val_max - color_val_min);
	color.b = color_val_min +
					(static_cast<double>(rand())/static_cast<double>(RAND_MAX))
					* (color_val_max - color_val_min);
	marker.color = color;


	// filling points
	Cloud points;
	pcl::fromPCLPointCloud2(mesh.cloud,points);
	for(int i = 0; i < mesh.polygons.size(); i++)
	{
		const pcl::Vertices &v =  mesh.polygons[i];
		for(int j = 0;j < v.vertices.size(); j++)
		{
			uint32_t index = v.vertices[j];
			geometry_msgs::Point p;
			p.x = points.points[index].x;
			p.y = points.points[index].y;
			p.z = points.points[index].z;
			marker.points.push_back(p);
		}
	}
}

void SurfaceDetection::add_cloud(Cloud& cloud)
{
	if(params_.use_octomap)
	{
		Cloud::iterator i;
		for( int i = 0
				; i < cloud.points.size();i++)
		{
			const pcl::PointXYZ &p = cloud.points[i];
			octomap::point3d entry = octomath::Vector3(p.x,p.y,p.z);
			octree_->updateNode(entry,true,true);
		}
		octree_->updateInnerOccupancy();
		ROS_INFO_STREAM("Aggregated new cloud to octomap");
	}
	else
	{
		(*full_cloud_ptr_)+=cloud;
		ROS_INFO_STREAM("Concatenated new cloud to acquired clouds");
	}
	acquired_clouds_counter_++;
	ROS_INFO_STREAM("Surface Detection is currently holding "<<acquired_clouds_counter_<<" point clouds");
}

int SurfaceDetection::get_acquired_clouds_count()
{
	return acquired_clouds_counter_;
}

void SurfaceDetection::process_octree()
{
	if(params_.use_octomap)
	{
		Cloud::Ptr buffer_cloud_ptr(new Cloud());
		buffer_cloud_ptr->reserve(octree_->getNumLeafNodes());
		full_cloud_ptr_->clear();
		octomap::OcTree::tree_iterator i;

		ROS_INFO_STREAM("Searching voxels with threshold > "<<octree_->getOccupancyThres());
		for(i = octree_->begin_tree(); i != octree_->end_tree(); i++)
		{

			if(octree_->isNodeOccupied(*i))
			{
				pcl::PointXYZ p;
				p.x = i.getX();
				p.y = i.getY();
				p.z = i.getZ();
				buffer_cloud_ptr->push_back(p);
			}
		}

		if(buffer_cloud_ptr->size() > 0)
		{
			pcl::copyPointCloud(*buffer_cloud_ptr,*full_cloud_ptr_);
			full_cloud_ptr_->header.frame_id = params_.frame_id;

		}

		ROS_INFO_STREAM("Total voxels found: "<<buffer_cloud_ptr->size());
	}
	else
	{

	}

}

visualization_msgs::MarkerArray SurfaceDetection::get_surface_markers()
{
	return visualization_msgs::MarkerArray(meshes_);
}

std::vector<Cloud::Ptr> SurfaceDetection::get_surface_clouds()
{
	return surface_clouds_;
}

std::string SurfaceDetection::get_results_summary()
{
	std::stringstream ss;
	if(surface_clouds_.size() > 0)
	{
		ss<<"\nNumber of surfaces identified: "<<surface_clouds_.size()<<"\n";
		for(int i =0;i < surface_clouds_.size(); i++)
		{
			ss<<"\t-segment "<<i+1<<" {points: "<<surface_clouds_[i]->size()<<"}\n";
		}

	}
	else
	{
		ss<<"\nNo surfaces have been found\n";
	}

	return ss.str();
}

void SurfaceDetection::get_full_cloud(Cloud& cloud)
{
	pcl::copyPointCloud(*full_cloud_ptr_,cloud);

}

void SurfaceDetection::get_full_cloud(sensor_msgs::PointCloud2 cloud_msg)
{
	pcl::toROSMsg(*full_cloud_ptr_,cloud_msg);
}

void SurfaceDetection::get_region_colored_cloud(CloudRGB& cloud )
{
	pcl::copyPointCloud(*region_colored_cloud_ptr_,cloud);
	cloud.header.frame_id = params_.frame_id;
}

void SurfaceDetection::get_region_colored_cloud(sensor_msgs::PointCloud2 &cloud_msg)
{
	pcl::toROSMsg(*region_colored_cloud_ptr_,cloud_msg);
	cloud_msg.header.frame_id = params_.frame_id;
}

bool SurfaceDetection::find_surfaces()
{
	if(full_cloud_ptr_->empty())
	{
		return false;
	}

	// process acquired sensor data
	process_octree();

	// reset members
	region_colored_cloud_ptr_ = CloudRGB::Ptr(new CloudRGB());
	surface_clouds_.clear();
	meshes_.markers.clear();

	// variables to hold intermediate results
	Normals::Ptr normals(new Normals());
	std::vector<pcl::PointIndices> clusters_indices;
	std::vector<Normals::Ptr> segment_normals;

	if(!params_.use_octomap )
	{
		if(apply_voxel_downsampling(*full_cloud_ptr_))
		{
			ROS_INFO_STREAM("Voxel downsampling succeeded, downsampled cloud size: "<<
					full_cloud_ptr_->size());
		}
		else
		{
			ROS_WARN_STREAM("Voxel downsampling failed, cloud size :"<<full_cloud_ptr_->size());
		}
	}

	if(params_.use_tabletop_seg)
	{
		int count = full_cloud_ptr_->size();
		if(apply_tabletop_segmentation(*full_cloud_ptr_,*full_cloud_ptr_))
		{
			ROS_INFO_STREAM("Tabletop segmentation successfully applied, new point count: "<<full_cloud_ptr_->size()
					<<", old point cloud: "<<count);
		}
		else
		{
			ROS_WARN_STREAM("Tabletop segmentation failed, ignoring results");
		}
	}
	else
	{
		ROS_WARN_STREAM("Tabletop segmentation skipped");
	}

	// apply statistical filter
	if(apply_statistical_filter(*full_cloud_ptr_,*full_cloud_ptr_))
	{
		ROS_INFO_STREAM("Statistical filter succeeded");
	}
	else
	{
		ROS_ERROR_STREAM("Statistical filter failed");
		return false;
	}

	// estimate normals
	if(apply_normal_estimation(*full_cloud_ptr_,*normals))
	{
		ROS_INFO_STREAM("Normal estimation succeeded");
	}
	else
	{
		ROS_ERROR_STREAM("Normal estimation failed");
		return false;
	}

	// applying region growing segmentation
	if(apply_region_growing_segmentation(*full_cloud_ptr_,*normals,clusters_indices,
			*region_colored_cloud_ptr_))
	{

		ROS_INFO_STREAM("Region growing succeeded");
		ROS_INFO_STREAM("Total surface clusters found: "<<clusters_indices.size());

		// filling cloud array
		for(int i =0;i< clusters_indices.size(); i++)
		{
			if(clusters_indices[i].indices.size() < params_.rg_min_cluster_size)
			{
				continue;
			}

			Cloud::Ptr segment_cloud_ptr(new Cloud());
			Normals::Ptr segment_normal_ptr(new Normals());

			// apply smoothing
			pcl::copyPointCloud(*full_cloud_ptr_,clusters_indices[i],
					*segment_cloud_ptr);

			int count = segment_cloud_ptr->size();
			if(apply_mls_surface_smoothing(*segment_cloud_ptr,*segment_cloud_ptr,*segment_normal_ptr))
			{
				ROS_INFO_STREAM("Moving least squares smoothing applied successfully for cluster "<<i<<". Count at t0: "<<count
						<<", Count at tf: "<<segment_cloud_ptr->size());
			}
			else
			{
				ROS_WARN_STREAM("Moving least squares smoothing failed for cluster "<<i<<", using original estimated normals");
				pcl::copyPointCloud(*normals,clusters_indices[i],*segment_normal_ptr);
			}

			segment_cloud_ptr->header.frame_id = params_.frame_id;
			surface_clouds_.push_back(segment_cloud_ptr);
			segment_normals.push_back(segment_normal_ptr);
		}

		ROS_INFO_STREAM("Selected surface clusters (> "<<params_.rg_min_cluster_size<<" points ) found: "<<surface_clouds_.size());
	}
	else
	{
		ROS_ERROR_STREAM("Region growing failed");
		return false;
	}

	if(params_.ignore_largest_cluster && surface_clouds_.size() > 1)
	{
		int largest_index = 0;
		int largest_size = 0;
		for(int i = 0;i < surface_clouds_.size();i++)
		{
			if(surface_clouds_[i]->points.size() > largest_size)
			{
				largest_size = surface_clouds_[i]->points.size();
				largest_index = i;
			}
		}

		ROS_INFO_STREAM("Removing larges cluster from results: cluster index [ "<<
				largest_index<<" ], cluster size [ "<<largest_size<<" ]");
		surface_clouds_.erase(surface_clouds_.begin() + largest_index);
		segment_normals.erase(segment_normals.begin() + largest_index);
	}

	// applying fast triangulation

	ROS_INFO_STREAM("Triangulation of surfaces started");
	for(int i = 0; i < surface_clouds_.size(); i++)
	{
		pcl::PolygonMesh mesh;
		visualization_msgs::Marker marker;
		apply_fast_triangulation(*surface_clouds_[i],*segment_normals[i],mesh);
		mesh_to_marker(mesh,marker);
		marker.header.frame_id = surface_clouds_[i]->header.frame_id;
		marker.id = i;
		marker.color.a = params_.marker_alpha;
		meshes_.markers.push_back(marker);
	}
	ROS_INFO_STREAM("Triangulation of surfaces completed");

	return true;
}

bool SurfaceDetection::apply_statistical_filter(const Cloud& in,Cloud& out)
{
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> filter;
	Cloud::ConstPtr cloud_ptr = boost::make_shared<Cloud>(in);
	filter.setInputCloud(cloud_ptr);
	filter.setMeanK(params_.meanK);
	filter.setStddevMulThresh(params_.stdv_threshold);
	filter.filter(out);

	return !out.empty();
}

bool SurfaceDetection::apply_normal_estimation(const Cloud &cloud,Normals& normals)
{
	Cloud::ConstPtr cloud_ptr = boost::make_shared<Cloud>(cloud);
	pcl::search::Search<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	pcl::NormalEstimation<pcl::PointXYZ,pcl::Normal> normal_estimator;
	normal_estimator.setViewPoint(0,0,5.0f);
	normal_estimator.setSearchMethod(tree);
	normal_estimator.setInputCloud(cloud_ptr);
	normal_estimator.setKSearch(params_.k_search);
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
	rg.setMinClusterSize(params_.rg_min_cluster_size);
	rg.setMaxClusterSize(params_.rg_max_cluster_size);
	rg.setSearchMethod(tree);
	rg.setNumberOfNeighbours(params_.rg_neightbors);
	rg.setInputCloud(cloud_ptr);
	rg.setInputNormals(normals_ptr);
	rg.setSmoothnessThreshold(params_.rg_smoothness_threshold);
	rg.setCurvatureThreshold(params_.rg_curvature_threshold);
	rg.extract(clusters);

	if(rg.getColoredCloud() != 0)
	{
		pcl::copyPointCloud(*rg.getColoredCloud(),colored_cloud);
	}


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
	gt.setSearchRadius(params_.tr_search_radius);
	gt.setMu(params_.tr_mu);
	gt.setMaximumNearestNeighbors(params_.tr_max_nearest_neighbors);
	gt.setMaximumSurfaceAngle(params_.tr_max_surface_angle);
	gt.setMinimumAngle(params_.tr_min_angle);
	gt.setMaximumAngle(params_.tr_max_angle);
	gt.setNormalConsistency(params_.tr_normal_consistency);

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
	vg.setLeafSize(params_.voxel_leafsize,params_.voxel_leafsize,params_.voxel_leafsize);
	vg.filter(*pcl_cloud_ptr);
	pcl::fromPCLPointCloud2(*pcl_cloud_ptr,cloud);
	return true;
}

bool SurfaceDetection::apply_mls_surface_smoothing(const Cloud& cloud_in,Cloud& cloud_out,Normals& normals)
{
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointNormal> mls_points;
	pcl::MovingLeastSquares<pcl::PointXYZ,pcl::PointNormal> mls;
	mls.setInputCloud(boost::make_shared<Cloud>(cloud_in));
	mls.setComputeNormals(true);
	mls.setPolynomialFit(true);

	mls.setUpsamplingMethod(pcl::MovingLeastSquares<pcl::PointXYZ,pcl::PointNormal>::RANDOM_UNIFORM_DENSITY);
	mls.setUpsamplingRadius(params_.mls_upsampling_radius);
	mls.setPointDensity(params_.mls_point_density);

	mls.setSearchMethod(tree);
	mls.setSearchRadius(params_.mls_search_radius);
	mls.process(mls_points);

	bool succeeded = mls_points.size() > 0;
	if(succeeded)
	{
		cloud_out.clear();
		normals.clear();
		pcl::copyPointCloud(mls_points,cloud_out);
		pcl::copyPointCloud(mls_points,normals);
	}
	return succeeded;
}

bool SurfaceDetection::apply_tabletop_segmentation(const Cloud& cloud_in,Cloud& cloud_out)
{
	pcl::ModelCoefficients::Ptr coeff_ptr(new pcl::ModelCoefficients());
	pcl::PointIndices::Ptr inliers_ptr(new pcl::PointIndices());
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	seg.setOptimizeCoefficients(true);
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setMaxIterations (1000);
	seg.setDistanceThreshold(params_.tabletop_seg_distance_threshold);
	seg.setInputCloud(boost::make_shared<Cloud>(cloud_in));
	seg.segment(*inliers_ptr,*coeff_ptr);

	bool succeeded = inliers_ptr->indices.size() > 0;
	if(succeeded)
	{
		pcl::ExtractIndices<pcl::PointXYZ> extract;
		extract.setInputCloud(boost::make_shared<Cloud>(cloud_in));
		extract.setIndices(inliers_ptr);
		extract.setNegative(true);
		extract.filter(cloud_out);
	}

	return succeeded;
}


}} /* namespace godel_surface_detection */
