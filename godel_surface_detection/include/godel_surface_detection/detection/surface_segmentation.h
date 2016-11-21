#ifndef SURFACE_SEGMENTATION_H
#define SURFACE_SEGMENTATION_H

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/PolygonMesh.h>
#include <pcl/geometry/mesh_base.h>
#include <pcl/geometry/triangle_mesh.h>
#include <pcl/geometry/quad_mesh.h>
#include <pcl/geometry/polygon_mesh.h>
#include <pcl/geometry/mesh_conversion.h>
#include <pcl/features/boundary.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/surface/ear_clipping.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/filters/filter.h>
#include <boost/foreach.hpp>
#include <Eigen/Geometry>
#include <Eigen/StdVector>


template <bool IsManifoldT>
struct MeshTraits
{
  typedef pcl::PointXYZRGBNormal                       VertexData;
  typedef pcl::geometry::NoData                        HalfEdgeData;
  typedef pcl::geometry::NoData                        EdgeData;
  typedef pcl::geometry::NoData                        FaceData;
  typedef boost::integral_constant <bool, IsManifoldT> IsManifold;
};

typedef MeshTraits <true > ManifoldMeshTraits;
typedef pcl::geometry::PolygonMesh <ManifoldMeshTraits> Mesh;
typedef typename Mesh::HalfEdgeIndex                     HalfEdgeIndex;
typedef typename Mesh::HalfEdgeIndices                   HalfEdgeIndices;
typedef typename Mesh::InnerHalfEdgeAroundFaceCirculator IHEAFC;


/** @class world_background_subtraction
@brief Maintains record of baseline sensor data to provide method to remove them leaving only new objects in the scene
*/
class surfaceSegmentation
{
  public:

  // mesh results
  pcl::PolygonMesh triangles_;
  std::vector<int> parts_;
  std::vector<int> states_;

  // segmentation results
  std::vector <pcl::PointIndices> clusters_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_;
  Mesh HEM_;

  // smoothing filter
  int num_coef_;
  std::vector<double> coef_;
  double gain_;

  // search terms
  double radius_;

  /** @brief default constructor */
  surfaceSegmentation()
  {
    // initialize pointers to cloud members
    input_cloud_= pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
  }

  /** @brief distructor */
  ~surfaceSegmentation()
  {
    input_cloud_ =  pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    normals_ =  pcl::PointCloud<pcl::Normal>::Ptr(new pcl::PointCloud<pcl::Normal>);
    input_cloud_->clear();
  }

  /** @brief constructor that sets the background cloud, also initializes the KdTree for searching
  @param bg_cloud the set of points defining the background
  */
  surfaceSegmentation(pcl::PointCloud<pcl::PointXYZ>::Ptr icloud)
  {
    input_cloud_ =  pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    normals_ =  pcl::PointCloud<pcl::Normal>::Ptr(new pcl::PointCloud<pcl::Normal>);
    setInputCloud(icloud);
    removeNans();
    computeNormals();
  }

  /** @brief sets the background cloud, replaces whatever points exists if any
  @param background_cloud the cloud representing the background
  */
  void setInputCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr icloud)
  {
    input_cloud_->clear();
    BOOST_FOREACH(pcl::PointXYZ pt, *icloud)
    {
      input_cloud_->push_back(pt);
    }
    computeNormals();
  }

  /** @brief adds new points to the background, and reinitializes the kd_tree for searching
  @param bg_cloud additional background points
  */
  void addCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr icloud)
  {
    // push input_cloud onto icloud and then add, this strange sequence keeps ordering of clouds and does not duplicate setInputCloud code
    BOOST_FOREACH(pcl::PointXYZ pt, input_cloud_->points)
    {
      icloud->push_back(pt);
    }
    setInputCloud(icloud);
  }

  /** @brief creates a cloud from every point estimated to be on the boundary of input_cloud_
  @return a boundary point cloud
  */
  pcl::PointCloud<pcl::Boundary>::Ptr getBoundaryCloud()
  {
    pcl::PointCloud<pcl::Boundary>::Ptr bps (new pcl::PointCloud<pcl::Boundary> ());
    if(normals_->points.size()==0 || input_cloud_->points.size() == 0)
      pcl::console::print_highlight ("must set input_cloud_ and compute normals_ before calling getBoundaryCloud()\n");

    else
    {
      pcl::BoundaryEstimation<pcl::PointXYZ, pcl::Normal, pcl::Boundary> best;
      best.setInputCloud(input_cloud_);
      best.setInputNormals(normals_);
      best.setRadiusSearch (radius_);
      //	best.setAngleThreshold(90.0*3.14/180.0);
      best.setSearchMethod (pcl::search::KdTree<pcl::PointXYZ>::Ptr (new pcl::search::KdTree<pcl::PointXYZ>));
      best.compute(*bps);
    }

    return(bps);
  }

  std::vector <pcl::PointIndices> computeSegments(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &colored_cloud)
  {
    // Region growing
    pcl::search::Search<pcl::PointXYZ>::Ptr tree =
        boost::shared_ptr<pcl::search::Search<pcl::PointXYZ>> (new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> rg;
    rg.setSmoothModeFlag (false); // Depends on the cloud being processed
    rg.setSmoothnessThreshold (10.0 / 180.0 * M_PI);
    rg.setMaxClusterSize(1000000);
    rg.setSearchMethod (tree);
    rg.setMinClusterSize(10);
    rg.setNumberOfNeighbours (30);
    float smooth_thresh = rg.getSmoothnessThreshold();
    float resid_thresh = rg.getResidualThreshold();
    // rg.setCurvatureTestFlag();
    rg.setResidualTestFlag(true);
    rg.setResidualThreshold(resid_thresh);
    rg.setCurvatureThreshold(1.0);

    rg.setInputCloud (input_cloud_);
    rg.setInputNormals (normals_);

    rg.extract (clusters_);
    colored_cloud = rg.getColoredCloud();
    return(clusters_);
  }


  /** @brief computes mesh on the cloud results are in triangles_, parts_, and states_ */
  Mesh computeMesh()
  {
    // use gpg3 to create a mesh, then traverse boundary of mesh to get boundaries
    // concatenate rgb fields and normals to cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_with_colors(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    BOOST_FOREACH(pcl::PointXYZ pt, *input_cloud_)
    {
      pcl::PointXYZRGB npt(0,255,0);
      npt.x = pt.x;
      npt.y = pt.y;
      npt.z = pt.z;
      cloud_with_colors->push_back(npt);
    }
    pcl::concatenateFields (*cloud_with_colors, *normals_, *cloud_with_normals);

    // Initialize objects
    pcl::GreedyProjectionTriangulation<pcl::PointXYZRGBNormal> gp3;
    // Create search tree*
    pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGBNormal>);
    tree->setInputCloud (cloud_with_normals);

    // Set typical values for the parameters
    gp3.setSearchRadius (radius_);
    gp3.setMu (2.5);
    gp3.setMaximumNearestNeighbors (500);
    gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
    gp3.setMinimumAngle(M_PI/18); // 10 degrees
    gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
    gp3.setNormalConsistency(false);
    gp3.setSearchMethod (tree);
    gp3.setInputCloud (cloud_with_normals);

    // Get results
    gp3.reconstruct (triangles_);
    parts_ = gp3.getPartIDs();
    states_ = gp3.getPointStates();

    pcl::console::print_highlight ("convert to half edge mesh\n");
    pcl::geometry::toHalfEdgeMesh(triangles_, HEM_);

    return(HEM_);
  }

    std::pair<int, int> getNextUnused(std::vector< std::pair<int,int> > used)
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
        //	  if(rtn.second ==0)     pcl::console::print_highlight ("returning 0 with i=%d\n",i);
        break;
        }
      }

      return(rtn);
    }

  int sortBoundary(pcl::IndicesPtr& boundary_indices, std::vector<pcl::IndicesPtr> &sorted_boundaries)
  {
    int longest = 0;
    int long_idx = 0;
    int length=0;
    ROS_INFO_STREAM("In sort boundary");
    for(int i=0;i<sorted_boundaries.size();i++)
      sorted_boundaries[i]->clear();

    sorted_boundaries.clear();


    /* initialize used pairs */
    std::vector<std::pair<int,int> > used;
    used.reserve(boundary_indices->size());
    ROS_INFO_STREAM("Used pairs reserved");
    for(int i = 0; i < boundary_indices->size(); i++)
    {
      std::pair<int, int> p;
      p.first = 0;
      p.second = boundary_indices->at(i);
      used.push_back(p);
    }

    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree(true);// true indicates return sorted radius search results
    kdtree.setInputCloud(input_cloud_, boundary_indices); // use just the boundary points for searching
    ROS_INFO_STREAM("Kdtree input cloud set");


    std::pair<int, int> n = getNextUnused(used);
    while( n.first >= 0 )
    {
      // add first point to the current boundary
      pcl::IndicesPtr current_boundary(new std::vector<int>);
      current_boundary->push_back(n.second);

      // find all points within small radius of current boundary point
      std::vector<int> pt_indices;
      std::vector<float> pt_dist;
      pcl::PointXYZ spt = input_cloud_->points[n.second];

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

        } while (q<pt_indices.size() && add_pt_idx == -1); // is there an unused point in the vicinity of the current spt?

        if(add_pt_idx !=-1)
        {
          current_boundary->push_back(add_pt_idx);
          spt = input_cloud_->points[add_pt_idx]; // search near the new point next time
        } // end if ad_pt_idx was found
        else
          break;		/* end of boundary */
      }// there are points within the radius

      sorted_boundaries.push_back(current_boundary);
      n = getNextUnused(used);
    }
    return(sorted_boundaries.size());
  }


  void setSearchRadius(double radius)
  {
    if(radius>0)
      radius_ = radius;
  }


  double  getSearchRadius()
  {
    return(radius_);
  }


  bool setSmoothCoef(std::vector<double> &coef)
  {
    if(coef.size() % 2 == 1)
    {		// smoothing filters must have an odd number of coefficients
      coef_.clear();
      num_coef_ = coef.size();
      double sum =0;
      for(int i=0; i<num_coef_; i++)
      {
        coef_.push_back(coef[i]);
        sum += coef[i];
      }
      gain_ = sum;		// set gain to be the sum of the coefficients because we need unity gain
      return(true);
    }
    else
    {
      return(false);
    }
  }


  void smoothVector(std::vector<double>&x_in, std::vector<double> &x_out)
  {
    int n = x_in.size();

    // initialize the filter using tail of x_in
    std::vector<double> xv;
    xv.clear();
    for(int j=num_coef_-1; j>=0; j--)
      xv.push_back(x_in[n-j-1]);

    // cycle through every and apply the filter
    for(int j=1; j<n-1; j++)
    {
      // shift backwards
      for(int k=0; k<num_coef_-1; k++)
        xv[k] = xv[k+1];

      // get next input to filter which is num_coef/2 in front of current point being smoothed
      int idx = (j+num_coef_/2)%n;
      //      pcl::console::print_highlight ("idx = %d\n", idx);
      xv[num_coef_ - 1] = x_in[idx]; // j'th point

    // apply the filter
    double sum = 0.0;
    for(int k=0; k<num_coef_; k++)
      sum += xv[k]*coef_[k];

    // save point
    x_out.push_back(sum/gain_);

    }// end for every point
  }


  void smoothPointNormal(std::vector<pcl::PointNormal> &pts_in,
                         std::vector<pcl::PointNormal> &pts_out)
  {
    std::vector<double> x_in, x_out, y_in, y_out, z_in, z_out;
    std::vector<double> nx_in, nx_out, ny_in, ny_out, nz_in, nz_out;
    for(int i=0;i<pts_in.size(); i++)
    {
      x_in.push_back(pts_in[i].x);
      y_in.push_back(pts_in[i].y);
      z_in.push_back(pts_in[i].z);
      nx_in.push_back(pts_in[i].normal_x);
      ny_in.push_back(pts_in[i].normal_y);
      nz_in.push_back(pts_in[i].normal_z);
    }
    smoothVector(x_in,x_out);
    smoothVector(y_in,y_out);
    smoothVector(z_in,z_out);
    smoothVector(nx_in,nx_out);
    smoothVector(ny_in,ny_out);
    smoothVector(nz_in,nz_out);

    pts_out.clear();
    for(int i=0;i<pts_in.size(); i++)
    {
      pcl::PointNormal pt;
      pt.x = x_out[i];
      pt.y = y_out[i];
      pt.z = z_out[i];
      double norm = sqrt(nx_out[i]*nx_out[i] + ny_out[i]*ny_out[i] + nz_out[i]*nz_out[i]);
      pt.normal_x = nx_out[i]/norm;
      pt.normal_y = ny_out[i]/norm;
      pt.normal_z = nz_out[i]/norm;
      pts_out.push_back(pt);
    }
  }


  void getBoundaryTrajectory(std::vector<pcl::IndicesPtr> &boundaries,
                             int sb,
                             std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>> &poses)
  {
    // grab the position and normal values
    std::vector<pcl::PointNormal> pts,spts;
    for(int i=0;i<boundaries[sb]->size();i++)
    {
      pcl::PointNormal pt;
      int idx = boundaries[sb]->at(i);
      pt.x = input_cloud_->points[idx].x;
      pt.y = input_cloud_->points[idx].y;
      pt.z = input_cloud_->points[idx].z;
      pt.normal_x = normals_->at(idx).normal_x;
      pt.normal_y = normals_->at(idx).normal_y;
      pt.normal_z = normals_->at(idx).normal_z;
      pts.push_back(pt);
    }
    smoothPointNormal(pts, spts);

    std::vector<pcl::PointXYZ> vels;
    for(int i=0;i<pts.size();i++)
    {
      pcl::PointXYZ v;
      int next = (i+1)%pts.size();
      v.x = pts[next].x-pts[i].x;
      v.y = pts[next].y-pts[i].y;
      v.z = pts[next].z-pts[i].z;
      double norm = sqrt(v.x*v.x + v.y*v.y + v.z*v.z);
      if(norm==0) norm = 1.0;	/* avoid division by zero */
      v.x = v.x/norm;
      v.y = v.y/norm;
      v.z = v.z/norm;
      vels.push_back(v);
    }

    poses.clear();
    for(int i=0;i<pts.size();i++)
    {
      Eigen::Matrix4d current_pose = Eigen::MatrixXd::Identity(4,4);
      current_pose(0,3) = pts[i].x;
      current_pose(1,3) = pts[i].y;
      current_pose(2,3) = pts[i].z;

      // set z vector same as of normal of surface
      current_pose(0,2) = -pts[i].normal_x;
      current_pose(1,2) = -pts[i].normal_y;
      current_pose(2,2) = -pts[i].normal_z;

      // set x of tool in direction of motion
      current_pose(0,0) = vels[i].x;
      current_pose(1,0) = vels[i].y;
      current_pose(2,0) = vels[i].z;

      // y is the cross product of z with x
      current_pose(0,1) =  current_pose(1,2)*current_pose(2,0)  -  current_pose(1,0)*current_pose(2,2);
      current_pose(1,1) = -current_pose(0,2)*current_pose(2,0) + current_pose(0,0)*current_pose(2,2);
      current_pose(2,1) =  current_pose(0,2)*current_pose(1,0)  -  current_pose(0,0)*current_pose(1,2);
      poses.push_back(current_pose);
    }
  }


  bool applyConcaveHull(pcl::PointCloud<pcl::PointXYZ>::Ptr& in, pcl::PolygonMesh& mesh)
  {
    pcl::PolygonMesh::Ptr hull_mesh_ptr(new pcl::PolygonMesh);
    pcl::ConcaveHull<pcl::PointXYZ> chull;
    chull.setInputCloud(in);
    chull.setAlpha(500.0);
    chull.reconstruct(mesh);
    pcl::console::print_highlight ("mesh  has  %d polygons\n", mesh.polygons.size());

    return mesh.polygons.size() > 0;
  }


  void getBoundBoundaryHalfEdges (const Mesh &mesh,
                                  std::vector <Mesh::HalfEdgeIndices>& boundary_he_collection,
                                  const size_t  expected_size = 3)
  {

    boundary_he_collection.clear ();

    HalfEdgeIndices boundary_he; boundary_he.reserve (expected_size);
    std::vector <bool> visited (mesh.sizeEdges (), false);
    IHEAFC circ, circ_end;

    pcl::console::print_highlight ("looking at %d half edges\n", mesh.sizeHalfEdges());
    for (HalfEdgeIndex i (0); i<HalfEdgeIndex (mesh.sizeHalfEdges ()); ++i)
    {
      if (mesh.isBoundary (i) && !visited [pcl::geometry::toEdgeIndex (i).get ()])
      {
        boundary_he.clear ();

        circ     = mesh.getInnerHalfEdgeAroundFaceCirculator (i);
        circ_end = circ;
        do
        {
        visited [pcl::geometry::toEdgeIndex (circ.getTargetIndex ()).get ()] = true;
        boundary_he.push_back (circ.getTargetIndex ());
        } while (++circ != circ_end);

        boundary_he_collection.push_back (boundary_he);
      }
    }
  }



private:
  /** @brief remove any NAN points, otherwise many algorityms fail */
  void removeNans()
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr nonans_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    nonans_cloud->is_dense = false;
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud (*input_cloud_, *nonans_cloud, indices);
    setInputCloud(nonans_cloud);
  }


  /** @brief compute the normals and store in normals_, this is requried for both segmentation and meshing*/
  void computeNormals()
  {
    normals_->points.clear();
    // Estimate the normals
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud (input_cloud_);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
    ne.setSearchMethod (tree);
    //  ne.setRadiusSearch (radius_);
    ne.setKSearch (50);
    ne.compute (*normals_);
  }

  pcl::PointCloud<pcl::Normal>::Ptr normals_;
  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals_;

};
#endif
