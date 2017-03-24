#include <segmentation/surface_segmentation.h>
#include <pcl/common/distances.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/project_inliers.h>
#include <ros/io.h>
#include <thread>

static const double DOWNSAMPLING_LEAF = 0.005f;
static const double EDGE_SEARCH_RADIUS = 0.01;
static const double PLANE_INLIER_DISTANCE = 0.005;
static const double PLANE_INLIER_THRESHOLD = 0.8;


// Custom boundary estimation
#include "parallel_boundary.h"

SurfaceSegmentation::SurfaceSegmentation()
{
  // initialize pointers to cloud members
  input_cloud_= pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
}


SurfaceSegmentation::~SurfaceSegmentation()
{
  input_cloud_ =  pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
  normals_ =  pcl::PointCloud<pcl::Normal>::Ptr(new pcl::PointCloud<pcl::Normal>);
  input_cloud_->clear();
}


SurfaceSegmentation::SurfaceSegmentation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr icloud)
{
  input_cloud_ =  pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
  normals_ =  pcl::PointCloud<pcl::Normal>::Ptr(new pcl::PointCloud<pcl::Normal>);
  setInputCloud(icloud);
  removeNans();
  computeNormals();

}


void SurfaceSegmentation::setInputCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr icloud)
{
  input_cloud_->clear();
  pcl::copyPointCloud(*icloud, *input_cloud_);

  // downsampling cloud
  input_cloud_downsampled_.reset(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::copyPointCloud(*input_cloud_,*input_cloud_downsampled_);
  pcl::VoxelGrid<pcl::PointXYZ> vg;
  vg.setInputCloud(input_cloud_downsampled_);
  vg.setLeafSize (DOWNSAMPLING_LEAF,DOWNSAMPLING_LEAF,DOWNSAMPLING_LEAF);
  vg.filter(*input_cloud_downsampled_);

  // create kdtree for speeding up search queries
  kd_tree_.reset(new pcl::search::KdTree<pcl::PointXYZ>());
  kd_tree_->setInputCloud(input_cloud_downsampled_);

}


void SurfaceSegmentation::addCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr icloud)
{
  // push input_cloud onto icloud and then add, this strange sequence keeps ordering of clouds
  // and does not duplicate setInputCloud code
  for(const auto& pt : input_cloud_->points)
      icloud->push_back(pt);

  setInputCloud(icloud);
}


void SurfaceSegmentation::getBoundaryCloud(pcl::PointCloud<pcl::Boundary>::Ptr &boundary_cloud)
{
  if(normals_->points.size()==0 || input_cloud_->points.size() == 0)
  {
    ROS_INFO_STREAM("Must set input_cloud_ and compute normals_ before calling getBoundaryCloud()");
  }
  else
  {
    pcl::ParallelBoundaryEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::Boundary> best;
    best.setInputCloud(input_cloud_);
    best.setInputNormals(normals_);
    best.setRadiusSearch (radius_);
    best.setSearchMethod (pcl::search::KdTree<pcl::PointXYZRGB>::Ptr (new pcl::search::KdTree<pcl::PointXYZRGB>));
    best.compute(*boundary_cloud);
  }
}


std::vector <pcl::PointIndices> SurfaceSegmentation::computeSegments(pcl::PointCloud<pcl::PointXYZRGB>::Ptr
                                                                     &colored_cloud)
{
  // Region growing
  pcl::search::Search<pcl::PointXYZRGB>::Ptr tree =
      boost::shared_ptr<pcl::search::Search<pcl::PointXYZRGB>> (new pcl::search::KdTree<pcl::PointXYZRGB>);
  pcl::RegionGrowing<pcl::PointXYZRGB, pcl::Normal> rg;

  rg.setSmoothModeFlag (true); // Depends on the cloud being processed
  rg.setSmoothnessThreshold (0.035);
  rg.setCurvatureThreshold(1.0);

  rg.setMaxClusterSize(MAX_CLUSTER_SIZE);
  rg.setSearchMethod (tree);
  rg.setMinClusterSize(MIN_CLUSTER_SIZE);
  rg.setNumberOfNeighbours (NUM_NEIGHBORS);

  float resid_thresh = rg.getResidualThreshold();

  rg.setResidualTestFlag(true);
  rg.setResidualThreshold(resid_thresh);
  rg.setInputCloud (input_cloud_);
  rg.setInputNormals (normals_);

  rg.extract (clusters_);

  if (!clusters_.empty()) // Turns out getColoredCloud() returns a null ptr if it didn't segment any surfaces
    colored_cloud = rg.getColoredCloud();

  return(clusters_);
}


std::pair<int, int> SurfaceSegmentation::getNextUnused(std::vector< std::pair<int,int> > used)
{
  std::pair<int,int> rtn;
  rtn.first=-1;
  rtn.second=-1;
  for(int i=0;i<used.size();i++)
  {
    if(used[i].first == 0)
    {
      rtn = used[i];
      used[i].first = 1;
      break;
    }
  }

  return(rtn);
}


int SurfaceSegmentation::sortBoundary(pcl::IndicesPtr& boundary_indices,
                                      std::vector<pcl::IndicesPtr> &sorted_boundaries)
{
  sorted_boundaries.clear();

  /* initialize used pairs */
  std::vector<std::pair<int,int> > used;
  used.reserve(boundary_indices->size());
  for(int i = 0; i < boundary_indices->size(); i++)
  {
    std::pair<int, int> p;
    p.first = 0;
    p.second = boundary_indices->at(i);
    used.push_back(p);
  }

  pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree(true);// true indicates return sorted radius search results
  kdtree.setInputCloud(input_cloud_, boundary_indices); // use just the boundary points for searching

  std::pair<int, int> n = getNextUnused(used);
  while( n.first >= 0 )
  {
    // add first point to the current boundary
    pcl::IndicesPtr current_boundary(new std::vector<int>);
    current_boundary->push_back(n.second);

    // find all points within small radius of current boundary point
    std::vector<int> pt_indices;
    std::vector<float> pt_dist;
    pcl::PointXYZRGB spt = input_cloud_->points[n.second];

    while ( kdtree.radiusSearch (spt, radius_, pt_indices, pt_dist) > 1 )
    { // gives index into input_cloud_,
      int q = 0;
      int add_pt_idx = -1;
      do
      {
        // find closest unused point in vicinity
        int pair_index=0;

        // for each item in used list
        for(int i=0;i<used.size();i++)
        {
          // look for a match
          if(pt_indices[q] == used[i].second)
          {
            // see if match is used
            if(used[i].first ==0)
            {
              pair_index= i;
              used[i].first = 1; // mark it used
              add_pt_idx = used[i].second;
            }// used
            break;
          }// match indices
        }

        q++;

      } while (q < pt_indices.size() && add_pt_idx == -1); // unused point in the vicinity of the current spt?

      if(add_pt_idx !=-1)
      {
        current_boundary->push_back(add_pt_idx);
        spt = input_cloud_->points[add_pt_idx]; // search near the new point next time
      }
      else
      { // end if ad_pt_idx was found
        break;		/* end of boundary */
      }
    }// there are points within the radius

    sorted_boundaries.push_back(current_boundary);
    n = getNextUnused(used);
  }

  return(sorted_boundaries.size());
}


void SurfaceSegmentation::setSearchRadius(double radius)
{
  if(radius > 0)
    radius_ = radius;
}


double  SurfaceSegmentation::getSearchRadius()
{
  return(radius_);
}


void SurfaceSegmentation::smoothVector(const std::vector<double> &x_in,
                                       std::vector<double> &x_out,
                                       const std::vector<double> &smoothing_coef)
{
  int num_coef = smoothing_coef.size();
  if (num_coef == 0 || num_coef % 2 != 1 || x_in.size() == 0 || x_in.size() <= num_coef)
  {
    if(num_coef % 2 == 0)
      ROS_WARN_STREAM("Filter must have an odd number of coefficients. Vector not smoothed");

    x_out = x_in;
    return;
  }

  double gain = std::accumulate(smoothing_coef.begin(), smoothing_coef.end(), 0);
  int n = x_in.size();

  // initialize the filter using tail of x_in
  std::vector<double> xv;
  for(int j = num_coef - 1; j >= 0; j--)
    xv.push_back(x_in[n-j-1]);

  // cycle through every and apply the filter
  for(int j = 1; j < n - 1; j++)
  {
    // shift backwards
    for(int k = 0; k < num_coef - 1; k++)
      xv[k] = xv[k + 1];

    // get next input to filter which is num_coef/2 in front of current point being smoothed
    int idx = (j + num_coef / 2) % n;
    xv[num_coef - 1] = x_in[idx]; // j'th point

    // apply the filter
    double sum = 0.0;
    for(int k = 0; k < num_coef; k++)
      sum += xv[k] * smoothing_coef[k];

    // save point
    x_out.push_back(sum / gain);
  } // end for every point
}


void SurfaceSegmentation::smoothPointNormal(std::vector<pcl::PointNormal> &pts_in,
                                            std::vector<pcl::PointNormal> &pts_out,
                                            int p_length = 13,
                                            int w_length = 31)
{
  std::vector<double> x_in, x_out, y_in, y_out, z_in, z_out;
  std::vector<double> nx_in, nx_out, ny_in, ny_out, nz_in, nz_out;

  for(int i = 0; i < pts_in.size(); i++)
  {
    x_in.push_back(pts_in[i].x);
    y_in.push_back(pts_in[i].y);
    z_in.push_back(pts_in[i].z);
    nx_in.push_back(pts_in[i].normal_x);
    ny_in.push_back(pts_in[i].normal_y);
    nz_in.push_back(pts_in[i].normal_z);
  }

  // Initialize vectors to the proper length
  std::vector<double> position_smoother(p_length);
  std::vector<double> orientation_smoother(w_length);

  // Smoother uses a linearly weighted average for both look-ahead and look-behind
  for(int i = 1; i < p_length; i++)
    position_smoother.push_back((i < p_length/2) ? i : (p_length - i));

  for(int i = 1; i < w_length; i++)
    orientation_smoother.push_back((i < w_length/2) ? i : w_length - i);

  // Smooth Points
  smoothVector(x_in, x_out, position_smoother);
  smoothVector(y_in, y_out, position_smoother);
  smoothVector(z_in, z_out, position_smoother);
  smoothVector(nx_in, nx_out, orientation_smoother);
  smoothVector(ny_in, ny_out, orientation_smoother);
  smoothVector(nz_in, nz_out, orientation_smoother);

  // Normalize and push to output vectors
  for(int i = 0; i < pts_in.size(); i++)
  {
    pcl::PointNormal pt;
    pt.x = x_out[i];
    pt.y = y_out[i];
    pt.z = z_out[i];
    double norm = sqrt(nx_out[i] * nx_out[i] + ny_out[i] * ny_out[i] + nz_out[i] * nz_out[i]);
    pt.normal_x = nx_out[i] / norm;
    pt.normal_y = ny_out[i] / norm;
    pt.normal_z = nz_out[i] / norm;
    pts_out.push_back(pt);
  }
}


bool SurfaceSegmentation::regularizeNormals(const std::vector<pcl::PointNormal>& boundary_pts,
                                            std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>>& poses,
                                            double eps,
                                            double plane_max_dist,
                                            double plane_inlier_threshold)
{
  // aliases
  using PCLPoint = pcl::PointXYZRGB;
  using Cloud = pcl::PointCloud<PCLPoint>;
  using BasicCloud = pcl::PointCloud<pcl::PointXYZ>;
  using SearchTree = pcl::KdTreeFLANN<PCLPoint>;

  // create cloud of nearby points
  BasicCloud::Ptr bd_nearby_points(new BasicCloud());

  // ============================================================
  // find points in surface near boundary points
  pcl::PointIndices::Ptr nearest_inliers (new pcl::PointIndices);
  std::vector<int> nearest_indices;
  std::vector<float> nearest_sqrt_dist;
  BasicCloud::Ptr nearest_points(new BasicCloud());
  pcl::PointXYZ p;
  int found = 0;

  pcl::PointNormal p0 = boundary_pts[0];
  for(std::size_t i = 0; i < boundary_pts.size();i++)
  {
    const pcl::PointNormal& pf = boundary_pts[i];

    if(pcl::euclideanDistance(p0,pf) < eps)
    {
      continue;
    }
    else
    {
      p0 = pf;
    }

    p.x = pf.x;
    p.y = pf.y;
    p.z = pf.z;

    nearest_indices.clear();
    nearest_sqrt_dist.clear();
    nearest_points->clear();

    found = kd_tree_->radiusSearchT(p,2*eps,nearest_indices,nearest_sqrt_dist);// kd-tree was built with the downsampled cloud
    if(found == 0)
    {
      continue;
    }

    // inserting points into cloud
    pcl::copyPointCloud(*input_cloud_downsampled_,nearest_indices,*nearest_points);

    // adding to larget cloud
    bd_nearby_points->insert(bd_nearby_points->end(),nearest_points->begin(),nearest_points->end());

  }

  if(bd_nearby_points->empty())
  {
    ROS_WARN("Failed to find points near edge boundary points");
    return false;
  }

  // dowsampling
  pcl::VoxelGrid<pcl::PointXYZ> vg;
  vg.setInputCloud(bd_nearby_points);
  vg.setLeafSize (DOWNSAMPLING_LEAF,DOWNSAMPLING_LEAF,DOWNSAMPLING_LEAF);
  vg.filter(*bd_nearby_points);

  ROS_INFO("Found %i points near boundary",int(bd_nearby_points->size()));

  // ============================================================
  // proceed to estimate plane
  pcl::ModelCoefficients::Ptr plane_coeffs(new pcl::ModelCoefficients());
  plane_coeffs->values = {0,0,1,0};
  pcl::PointIndices::Ptr plane_inliers(new pcl::PointIndices());
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold(plane_max_dist);
  seg.setMaxIterations(100);
  seg.setInputCloud(bd_nearby_points);
  seg.segment(*plane_inliers,*plane_coeffs);

  ROS_INFO_STREAM("Surface Plane coefficients "<< plane_coeffs->values[0]<< ", "<<
                  plane_coeffs->values[1]<<", "<< plane_coeffs->values[2]);

  double inlier_percentage = double(plane_inliers->indices.size())/double(bd_nearby_points->size());
  if(inlier_percentage <= plane_inlier_threshold)
  {
    ROS_WARN("Only %f of the points near the boundary fall within %f to the surface plane, quitting due to being below threshold of %f",
              inlier_percentage,plane_max_dist,plane_inlier_threshold);
    return false;
  }

  ROS_INFO("Plane inliers near boundary amount to %f  ",100.0*inlier_percentage);

  // ============================================================
  // extracting plane normal
  Eigen::Vector3d z_vect(plane_coeffs->values[0],plane_coeffs->values[1],plane_coeffs->values[2]);
  z_vect.normalize();
  z_vect = (z_vect.z() > 0.0) ? z_vect : -z_vect; // inverting if z is negative


  // projecting points onto plane
  BasicCloud::Ptr proj_boundary_points(new BasicCloud());
  proj_boundary_points->reserve(boundary_pts.size());
  for(auto& pn : boundary_pts)
  {
    pcl::PointXYZ p;
    pcl::copyPoint(pn,p);
    proj_boundary_points->push_back(p);
  }

  pcl::ProjectInliers<pcl::PointXYZ> proj;
  proj.setModelType(pcl::SACMODEL_PLANE);
  proj.setInputCloud(proj_boundary_points);
  proj.setModelCoefficients(plane_coeffs);
  proj.filter(*proj_boundary_points);


  // ============================================================
  // apply plane normal to all boundary points
  Eigen::Vector3d x_vect, y_vect;
  Eigen::Matrix3d orient = Eigen::Matrix3d::Identity();
  poses.clear();
  Eigen::Affine3d pose;
  int look_ahead_ind;
  double dist = 0.0;
  x_vect = x_vect.UnitX();
  const int index_incr = 5;
  for(std::size_t i = 0; i < proj_boundary_points->size();i++)
  {
    const pcl::PointXYZ& p0 = proj_boundary_points->at(i);

    // computing x direction

    pcl::PointXYZ pf;
    dist = 0.0;
    look_ahead_ind = i+index_incr;
    if(look_ahead_ind < proj_boundary_points->size())
    {
      pf = proj_boundary_points->at(look_ahead_ind);
      dist = pcl::euclideanDistance(p0,pf);
      look_ahead_ind+=index_incr;
    }

    // constructing x vector
    if(dist > 1e-5)
    {
      x_vect.x() = pf.x - p0.x;
      x_vect.y() = pf.y - p0.y;
      x_vect.z() = pf.z - p0.z;
      x_vect.normalize();
    }

    y_vect = z_vect.cross(x_vect);
    y_vect.normalize();

    // ensuring orthogonality
    x_vect = y_vect.cross(z_vect);
    x_vect.normalize();

    orient.col(0) = x_vect;
    orient.col(1) = y_vect;
    orient.col(2) = z_vect;

    // constructing matrix
    pose.setIdentity();
    pose = Eigen::Translation3d(p0.x,p0.y,p0.z)*Eigen::Quaterniond(orient);

    // adding pose
    poses.push_back(pose.matrix());
  }

  ROS_INFO_STREAM("Regularized edge boundary normals to "<< z_vect.transpose());

  return true;
}


void SurfaceSegmentation::getBoundaryTrajectory(std::vector<pcl::IndicesPtr> &boundaries,
                                                int sb,
                                                std::vector<Eigen::Matrix4d,
                                                Eigen::aligned_allocator<Eigen::Matrix4d>> &poses)
{

  // grab the position and normal values
  std::vector<pcl::PointNormal> pts, spts;
  for(int i = 0; i < boundaries[sb]->size(); i++)
  {
    pcl::PointNormal pt;
    int idx = boundaries[sb]->at(i);
    pt.x = input_cloud_->points[idx].x;
    pt.y = input_cloud_->points[idx].y;
    pt.z = input_cloud_->points[idx].z;

    int sign_ofz=1;
    if(normals_->at(idx).normal_z < 0)
      sign_ofz = -1;

    pt.normal_x = sign_ofz * normals_->at(idx).normal_x;
    pt.normal_y = sign_ofz * normals_->at(idx).normal_y;
    pt.normal_z = sign_ofz * normals_->at(idx).normal_z;
    pts.push_back(pt);
  }

  smoothPointNormal(pts, spts);


  poses.clear();
  if(regularizeNormals(spts,poses,EDGE_SEARCH_RADIUS,PLANE_INLIER_DISTANCE,PLANE_INLIER_THRESHOLD))
  {
    return; // surface was sufficiently flat
  }
  ROS_WARN("Normal regularization wasn't feasible, defaulting to averaging approach");

  std::vector<pcl::PointXYZRGB> vels;
  for(int i = 0; i < spts.size(); i++)
  {
    pcl::PointXYZRGB v;
    int next = (i + 1) % pts.size();
    v.x = spts[next].x - spts[i].x;
    v.y = spts[next].y - spts[i].y;
    v.z = spts[next].z - spts[i].z;
    double norm = sqrt(v.x * v.x + v.y * v.y + v.z * v.z);

    if(norm == 0)
      norm = 1.0;	/* avoid division by zero */

    v.x = v.x / norm;
    v.y = v.y / norm;
    v.z = v.z / norm;
    vels.push_back(v);
  }

  // make sure normal and velocity vectors are orthogonal
  for(int i = 0; i < spts.size(); i++)
  {
     double dot = spts[i].normal_x * vels[i].x + spts[i].normal_y * vels[i].y + spts[i].normal_z * vels[i].z;
     vels[i].x -= dot * spts[i].normal_x;
     vels[i].y -= dot * spts[i].normal_y;
     vels[i].z -= dot * spts[i].normal_z;

     double norm = sqrt(vels[i].x * vels[i].x + vels[i].y * vels[i].y + vels[i].z * vels[i].z);

     if(norm == 0)
       norm = 1;

     vels[i].x = vels[i].x / norm;
     vels[i].y = vels[i].y / norm;
     vels[i].z = vels[i].z / norm;
  }


  for(int i = 0; i < spts.size(); i++)
  {
    Eigen::Matrix4d current_pose = Eigen::MatrixXd::Identity(4, 4);
    current_pose(0, 3) = spts[i].x;
    current_pose(1, 3) = spts[i].y;
    current_pose(2, 3) = spts[i].z;

    // set z vector same as of normal of surface
    current_pose(0, 2) = spts[i].normal_x;
    current_pose(1, 2) = spts[i].normal_y;
    current_pose(2, 2) = spts[i].normal_z;

    // set x of tool in direction of motion
    current_pose(0, 0) = vels[i].x;
    current_pose(1, 0) = vels[i].y;
    current_pose(2, 0) = vels[i].z;

    // y is the cross product of z with x
    current_pose(0, 1) =  current_pose(1, 2) * current_pose(2,0) - current_pose(1, 0) * current_pose(2, 2);
    current_pose(1, 1) = -current_pose(0, 2) * current_pose(2,0) + current_pose(0, 0) * current_pose(2, 2);
    current_pose(2, 1) =  current_pose(0, 2) * current_pose(1,0) - current_pose(0, 0) * current_pose(1, 2);
    poses.push_back(current_pose);
  }
}


void SurfaceSegmentation::removeNans()
{
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud (*input_cloud_, *input_cloud_, indices);
}


void SurfaceSegmentation::computeNormals()
{
  // Determine the number of available cores
  pcl::NormalEstimationOMP<pcl::PointXYZRGB, pcl::Normal> ne;
  int nr_cores = std::thread::hardware_concurrency();
  ne.setNumberOfThreads(nr_cores);

  // Configure parameters
  ne.setInputCloud (input_cloud_);
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
  ne.setSearchMethod (tree);
  ne.setRadiusSearch(0.025);
//  ne.setKSearch (100);

  // Estimate the normals
  ne.compute (*normals_);
}


void SurfaceSegmentation::getSurfaceClouds(std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> &surface_clouds)
{
  surface_clouds.clear();
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr segment_cloud_ptr;

  for (const auto& cluster : clusters_)
  {
    segment_cloud_ptr = pcl::PointCloud<pcl::PointXYZRGB>::Ptr (new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::PointIndices indices = cluster;
    if (indices.indices.size() == 0)
      continue;

    if (indices.indices.size() >= MIN_CLUSTER_SIZE)
    {
      pcl::copyPointCloud(*input_cloud_, indices, *segment_cloud_ptr);
      surface_clouds.push_back(segment_cloud_ptr);
    }
  }
}
