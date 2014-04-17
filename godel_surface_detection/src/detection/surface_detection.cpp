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
		frame_id_(defaults::FRAME_ID),
		k_search_(defaults::K_SEARCH),
		meanK_(defaults::STATISTICAL_OUTLIER_MEAN),
		stdv_threshold_(defaults::STATISTICAL_OUTLIER_STDEV_THRESHOLD),
		rg_min_cluster_size_(defaults::REGION_GROWING_MIN_CLUSTER_SIZE),
		rg_max_cluster_size_(defaults::REGION_GROWING_MAX_CLUSTER_SIZE),
		rg_neightbors_(defaults::REGION_GROWING_NEIGHBORS),
		rg_smoothness_threshold_(defaults::REGION_GROWING_SMOOTHNESS_THRESHOLD),
		rg_curvature_threshold_(defaults::REGION_GROWING_CURVATURE_THRESHOLD),
		//acquisition_time_(defaults::ACQUISITION_TIME),
		tr_search_radius_(defaults::TRIANGULATION_SEARCH_RADIUS),
		tr_mu_(defaults::TRIANGULATION_MU),
		tr_max_nearest_neighbors_(defaults::TRIANGULATION_MAX_NEAREST_NEIGHBORS),
		tr_max_surface_angle_(defaults::TRIANGULATION_MAX_SURFACE_ANGLE),
		tr_min_angle_(defaults::TRIANGULATION_MIN_ANGLE),
		tr_max_angle_(defaults::TRIANGULATION_MAX_ANGLE),
		tr_normal_consistency_(defaults::TRIANGULATION_NORMAL_CONSISTENCY),
		voxel_leafsize_(defaults::VOXEL_LEAF_SIZE),
		marker_alpha_(defaults::MARKER_ALPHA),
		ignore_largest_cluster_(defaults::IGNORE_LARGEST_CLUSTER),
		use_octomap_(defaults::USE_OCTOMAP),
		occupancy_threshold_(defaults::OCCUPANCY_THRESHOLD),
		mls_upsampling_radius_(defaults::MLS_UPSAMPLING_RADIUS),
		mls_point_density_(defaults::MLS_POINT_DENSITY),
		mls_search_radius_(defaults::MLS_SEARCH_RADIUS),
		use_tabletop_seg_(defaults::USE_TABLETOP_SEGMENTATION),
		tabletop_seg_distance_threshold_(defaults::TABLETOP_SEG_DISTANCE_THRESH),
		octree_(new octomap::OcTree(defaults::VOXEL_LEAF_SIZE)),
		full_cloud_ptr_(new Cloud())
{
	// TODO Auto-generated constructor stub
	srand(time(NULL));
}

SurfaceDetection::~SurfaceDetection()
{
	// TODO Auto-generated destructor stub
}

bool SurfaceDetection::init()
{
	octree_->setResolution(voxel_leafsize_);
	octree_->setOccupancyThres(occupancy_threshold_);
	full_cloud_ptr_->header.frame_id = frame_id_;
	acquired_clouds_counter_ = 0;
	return true;
}

bool SurfaceDetection::load_parameters(std::string node_ns)
{
	ros::NodeHandle nh(node_ns);

	bool succeeded;
	if(		nh.getParam(params::FRAME_ID,frame_id_)&&
			nh.getParam(params::STOUTLIER_MEAN,meanK_) &&
			nh.getParam(params::STOUTLIER_STDEV_THRESHOLD,stdv_threshold_) &&
			nh.getParam(params::REGION_GROWING_MIN_CLUSTER_SIZE,rg_min_cluster_size_) &&
			nh.getParam(params::REGION_GROWING_MAX_CLUSTER_SIZE,rg_max_cluster_size_) &&
			nh.getParam(params::REGION_GROWING_NEIGHBORS,rg_neightbors_) &&
			nh.getParam(params::REGION_GROWING_SMOOTHNESS_THRESHOLD,rg_smoothness_threshold_) &&
			nh.getParam(params::REGION_GROWING_CURVATURE_THRESHOLD,rg_curvature_threshold_) &&
			nh.getParam(params::TRIANGULATION_SEARCH_RADIUS,tr_search_radius_) &&
			nh.getParam(params::TRIANGULATION_MU ,tr_mu_) &&
			nh.getParam(params::TRIANGULATION_MAX_NEAREST_NEIGHBORS,tr_max_nearest_neighbors_) &&
			nh.getParam(params::TRIANGULATION_MAX_SURFACE_ANGLE,tr_max_surface_angle_) &&
			nh.getParam(params::TRIANGULATION_MIN_ANGLE,tr_min_angle_) &&
			nh.getParam(params::TRIANGULATION_MAX_ANGLE,tr_max_angle_) &&
			nh.getParam(params::TRIANGULATION_NORMAL_CONSISTENCY,tr_normal_consistency_) &&
			nh.getParam(params::VOXEL_LEAF_SIZE,voxel_leafsize_) &&
			nh.getParam(params::OCCUPANCY_THRESHOLD,occupancy_threshold_) &&
			nh.getParam(params::MLS_UPSAMPLING_RADIUS,mls_upsampling_radius_) &&
			nh.getParam(params::MLS_POINT_DENSITY,mls_point_density_) &&
			nh.getParam(params::MLS_SEARCH_RADIUS,mls_search_radius_) &&
			nh.getParam(params::USE_TABLETOP_SEGMENTATION,use_tabletop_seg_) &&
			nh.getParam(params::TABLETOP_SEG_DISTANCE_THRESH,tabletop_seg_distance_threshold_) &&
			nh.getParam(params::MARKER_ALPHA,marker_alpha_) &&
			nh.getParam(params::IGNORE_LARGEST_CLUSTER,ignore_largest_cluster_)
			)
	{
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
	for(int i = 0; i < mesh.polygons.size(); i++)
	{
		const pcl::Vertices &v =  mesh.polygons[i];
		Cloud points;
		pcl::fromPCLPointCloud2(mesh.cloud,points);
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
	if(use_octomap_)
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
}

void SurfaceDetection::process_octree()
{
	if(use_octomap_)
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
			full_cloud_ptr_->header.frame_id = frame_id_;

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
	cloud.header.frame_id = frame_id_;
}

void SurfaceDetection::get_region_colored_cloud(sensor_msgs::PointCloud2 &cloud_msg)
{
	pcl::toROSMsg(*region_colored_cloud_ptr_,cloud_msg);
	cloud_msg.header.frame_id = frame_id_;
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

	if(!use_octomap_ )
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

	if(use_tabletop_seg_)
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

		// filling cloud array
		for(int i =0;i< clusters_indices.size(); i++)
		{
			if(clusters_indices[i].indices.size() < rg_min_cluster_size_)
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

			segment_cloud_ptr->header.frame_id = frame_id_;
			surface_clouds_.push_back(segment_cloud_ptr);
			segment_normals.push_back(segment_normal_ptr);
		}

		ROS_INFO_STREAM("\nRegion growing succeeded:\n"<<
				"\tTotal surface clusters found: "<<clusters_indices.size()<<"\n"
				"\tValid surface clusters (> "<<rg_min_cluster_size_<<" points ) found: "<<surface_clouds_.size());
	}
	else
	{
		ROS_ERROR_STREAM("Region growing failed");
		return false;
	}

	if(ignore_largest_cluster_ && surface_clouds_.size() > 1)
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
		marker.color.a = marker_alpha_;
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
	rg.setSmoothnessThreshold(rg_smoothness_threshold_);
	rg.setCurvatureThreshold(rg_curvature_threshold_);
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
	vg.setLeafSize(voxel_leafsize_,voxel_leafsize_,voxel_leafsize_);
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
	mls.setUpsamplingRadius(mls_upsampling_radius_);
	mls.setPointDensity(mls_point_density_);

	mls.setSearchMethod(tree);
	mls.setSearchRadius(mls_search_radius_);
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
	seg.setDistanceThreshold(tabletop_seg_distance_threshold_);
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
