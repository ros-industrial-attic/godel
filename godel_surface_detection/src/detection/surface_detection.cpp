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
#include <tf/transform_datatypes.h>

namespace godel_surface_detection { namespace detection{

SurfaceDetection::SurfaceDetection():
		acquired_clouds_counter_(0),
		acquisition_topic_(config::POINT_COLUD_TOPIC),
		frame_id_(defaults::FRAME_ID),
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
		voxel_leafsize_(defaults::VOXEL_LEAF_SIZE),
		marker_alpha_(defaults::MARKER_ALPHA),
		ignore_largest_cluster_(defaults::IGNORE_LARGEST_CLUSTER)
{
	// TODO Auto-generated constructor stub
	srand(time(NULL));
}

SurfaceDetection::~SurfaceDetection()
{
	// TODO Auto-generated destructor stub
}

bool SurfaceDetection::init(std::string node_ns)
{
	return load_parameters(node_ns);
}

bool SurfaceDetection::load_parameters(std::string node_ns)
{
	ros::NodeHandle nh(node_ns.empty() ? "~" : node_ns);
	bool succeeded;
	const std::string ns = params::PARAMETER_NS + "/";
	if(nh.getParam(ns + params::ACQUISITION_TIME,acquisition_time_) &&
			nh.getParam(ns + params::FRAME_ID,frame_id_)&&
			nh.getParam(ns + params::STOUTLIER_MEAN,meanK_) &&
			nh.getParam(ns + params::STOUTLIER_STDEV_THRESHOLD,stdv_threshold_) &&
			nh.getParam(ns + params::REGION_GROWING_MIN_CLUSTER_SIZE,rg_min_cluster_size_) &&
			nh.getParam(ns + params::REGION_GROWING_MAX_CLUSTER_SIZE,rg_max_cluster_size_) &&
			nh.getParam(ns + params::REGION_GROWING_NEIGHBORS,rg_neightbors_) &&
			nh.getParam(ns + params::REGION_GROWING_SMOOTHNESS_THRESHOLD,rg_smoothness_threshold_) &&
			nh.getParam(ns + params::REGION_GROWING_CURVATURE_THRESHOLD,rg_curvature_threshold_) &&
			nh.getParam(ns + params::TRIANGULATION_SEARCH_RADIUS,tr_search_radius_) &&
			nh.getParam(ns + params::TRIANGULATION_MU ,tr_mu_) &&
			nh.getParam(ns + params::TRIANGULATION_MAX_NEAREST_NEIGHBORS,tr_max_nearest_neighbors_) &&
			nh.getParam(ns + params::TRIANGULATION_MAX_SURFACE_ANGLE,tr_max_surface_angle_) &&
			nh.getParam(ns + params::TRIANGULATION_MIN_ANGLE,tr_min_angle_) &&
			nh.getParam(ns + params::TRIANGULATION_MAX_ANGLE,tr_max_angle_) &&
			nh.getParam(ns + params::TRIANGULATION_NORMAL_CONSISTENCY,tr_normal_consistency_) &&
			nh.getParam(ns + params::VOXEL_LEAF_SIZE,voxel_leafsize_) &&
			nh.getParam(ns + params::MARKER_ALPHA,marker_alpha_) &&
			nh.getParam(ns + params::IGNORE_LARGEST_CLUSTER,ignore_largest_cluster_)
			)
	{
		succeeded = true;
		ROS_INFO_STREAM("surface detection parameters loaded");
	}
	else
	{
		succeeded = false;
		ROS_ERROR_STREAM("surface detection failed to load one or more parameters");
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

void SurfaceDetection::set_acquisition_time(double val)
{
	acquisition_time_ = val;
}

void SurfaceDetection::get_acquired_cloud(Cloud& cloud)
{
	pcl::copyPointCloud(*acquired_cloud_ptr_,cloud);
}

void SurfaceDetection::get_acquired_cloud(sensor_msgs::PointCloud2 cloud_msg)
{
	pcl::toROSMsg(*acquired_cloud_ptr_,cloud_msg);
}

void SurfaceDetection::get_region_colored_cloud(CloudRGB& cloud )
{
	pcl::copyPointCloud(*region_colored_cloud_ptr_,cloud);
	cloud.header.frame_id = acquired_cloud_ptr_->header.frame_id;
}

void SurfaceDetection::get_region_colored_cloud(sensor_msgs::PointCloud2 &cloud_msg)
{
	pcl::toROSMsg(*region_colored_cloud_ptr_,cloud_msg);
	cloud_msg.header.frame_id = acquired_cloud_ptr_->header.frame_id;
}

bool SurfaceDetection::acquire_data()
{
	// initialize  aggregate point cloud
	acquired_cloud_ptr_.reset(new Cloud());
	acquired_clouds_counter_ = 0;

	// initialize subscriber
	ros::NodeHandle nh;
	point_cloud_subs_ = nh.subscribe(acquisition_topic_,1,
			&SurfaceDetection::point_cloud_subscriber_cb,this);

	// wait for topic
	sensor_msgs::PointCloud2ConstPtr msg =
			ros::topic::waitForMessage<sensor_msgs::PointCloud2>(point_cloud_subs_.getTopic(),
					ros::Duration(5.0f));

	ros::AsyncSpinner spinner(2);
	spinner.start();

	if(msg)
	{
		ROS_INFO_STREAM("Started point cloud acquisition");

		ros::Duration acquistion_time(acquisition_time_);
		ros::Time start_time = ros::Time::now();
		while(ros::ok() &&
				((start_time + acquistion_time) > ros::Time::now()))
		{
			//ros::spinOnce();
		}

		point_cloud_subs_.shutdown();
		ROS_INFO_STREAM("Finished point cloud acquisition");
		spinner.stop();
		return !acquired_cloud_ptr_->empty();
	}
	else
	{
		ROS_ERROR_STREAM("Point cloud topic: '"<<point_cloud_subs_.getTopic()<<
				"' not advertised");
		spinner.stop();
		return false;
	}
}

bool SurfaceDetection::find_surfaces()
{
	// reset members
	region_colored_cloud_ptr_ = CloudRGB::Ptr(new CloudRGB());
	surface_clouds_.clear();
	meshes_.markers.clear();

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

		// filling cloud array
		for(int i =0;i< clusters_indices.size(); i++)
		{
			if(clusters_indices[i].indices.size() < rg_min_cluster_size_)
			{
				continue;
			}

			Cloud::Ptr segment_cloud_ptr(new Cloud());
			Normals::Ptr segment_normal_ptr(new Normals());
			pcl::copyPointCloud(*acquired_cloud_ptr_,clusters_indices[i],
					*segment_cloud_ptr);
			pcl::copyPointCloud(*normals,clusters_indices[i],*segment_normal_ptr);

			segment_cloud_ptr->header.frame_id = acquired_cloud_ptr_->header.frame_id;
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

void SurfaceDetection::point_cloud_subscriber_cb(const sensor_msgs::PointCloud2ConstPtr &msg)
{
	static tf::TransformListener tf_listener;

	// convert to message to point cloud
	Cloud::Ptr new_cloud_ptr(new Cloud());
	pcl::fromROSMsg<pcl::PointXYZ>(*msg,*new_cloud_ptr);

	// removed nans
	std::vector<int> index;
	pcl::removeNaNFromPointCloud(*new_cloud_ptr,*new_cloud_ptr,index);

	// transform to frame id
	tf::StampedTransform source_to_target_tf;
	if(frame_id_.compare(msg->header.frame_id) !=0)
	{
		ROS_INFO_STREAM("Source cloud with frame id '"<<msg->header.frame_id<<"' will be transformed to frame id: '"
				<<frame_id_<<"'");
		try
		{
			tf_listener.lookupTransform(frame_id_,msg->header.frame_id,
					ros::Time::now() - ros::Duration(0.2f),source_to_target_tf);
			pcl_ros::transformPointCloud(*new_cloud_ptr,*new_cloud_ptr,source_to_target_tf);
		}
		catch(tf::LookupException &e)
		{
			ROS_ERROR_STREAM("Transform lookup error, using source frame id '"<< msg->header.frame_id<<"'");
			frame_id_ = msg->header.frame_id;
		}
		catch(tf::ExtrapolationException &e)
		{
			ROS_ERROR_STREAM("Transform lookup error, using source frame id '"<< msg->header.frame_id<<"'");
			frame_id_ = msg->header.frame_id;
		}

	}
	else
	{
		ROS_INFO_STREAM("Source cloud is already in frame id '"<<msg->header.frame_id<<", skipping transform");
	}

	// add to aggregate point cloud
	(*acquired_cloud_ptr_)+=*new_cloud_ptr;
	acquired_clouds_counter_++;

	acquired_cloud_ptr_->header.frame_id = frame_id_;

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


}} /* namespace godel_surface_detection */
