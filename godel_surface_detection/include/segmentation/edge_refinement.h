/*
    TODO:
    - Check ENSENSO scan density and predict the amount of neighbors.
    - Add B-Spline Smoother:
      http://stackoverflow.com/questions/25379422/b-spline-curves
      http://kluge.in-chemnitz.de/opensource/spline/
      http://pointclouds.org/documentation/tutorials/bspline_fitting.php
*/

#ifndef EDGE_REFINEMENT_H
#define EDGE_REFINEMENT_H

#include <math.h>
#include <pcl/common/common.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/surface/gp3.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/features/boundary.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/visualization/pcl_visualizer.h>      

namespace godel_scan_tools
{
typedef std::vector<pcl::PointCloud<pcl::PointXYZ>, Eigen::aligned_allocator<pcl::PointXYZ>> PointCloudVector;
typedef std::vector<pcl::PointCloud<pcl::Boundary>, Eigen::aligned_allocator<pcl::Boundary>> PointCloudBoundaryVector;
typedef std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> EigenPoseMatrix;
typedef std::vector<pcl::PointXYZ> PointVector;
typedef Eigen::Matrix<float, 1, 3> NormalVector;
typedef Eigen::Matrix<float, 1, 3> PoseOrigin;

/**
 * @brief      Structure containing the data for the debug display.
 */
struct DebugDisplayData
{
  /**
   * @brief      Constructor for DebugDisplayData.
   *
   * @param[in]  current_pose_index              The current pose index
   * @param[in]  num_poses                       The number poses
   * @param      viewer                          The viewer
   * @param[in]  boundary_poses                  The boundary poses
   * @param[in]  boundary_pose_neighbor          The boundary pose neighbor
   * @param[in]  refined_boundary_pose_neighbor  The refined boundary pose neighbor
   * @param[in]  neighbor_boundary_points        The neighbor boundary points
   * @param[in]  new_pose_points                 The new pose points
   * @param[in]  additional_poses                The additional poses
   */
  DebugDisplayData(const std::size_t current_pose_index, const std::size_t num_poses, 
                   pcl::visualization::PCLVisualizer *viewer,
                   const EigenPoseMatrix boundary_poses, 
                   const PointCloudVector boundary_pose_neighbor, 
                   const PointCloudVector refined_boundary_pose_neighbor, 
                   const PointCloudVector neighbor_boundary_points,
                   const PointVector new_pose_points,
                   const std::map<int, PointVector> additional_poses);

  bool rendered_additional_shapes_;
  std::size_t rendered_shape_count_;
  std::size_t current_pose_index_;
  std::size_t num_poses_;
  pcl::visualization::PCLVisualizer *viewer_;

  EigenPoseMatrix boundary_poses_;
  PointCloudVector boundary_pose_neighbor_;
  PointCloudVector refined_boundary_pose_neighbor_;
  PointCloudVector neighbor_boundary_points_;
  PointVector new_pose_points_;
  std::map<int, PointVector> additional_poses_;

};

/**
 * @brief      Class for edge refinement.
 */
class EdgeRefinement
{
public:
  /**
   * @brief      Constructor for EdgeRefinement class.
   *
   * @param[in]  cloud  Original Point cloud data that does not contain any NaNs.
   */
  EdgeRefinement(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

  /**
   * @brief      Constructor for EdgeRefinement class with initializer list.
   */
  EdgeRefinement():tree_(new pcl::search::KdTree<pcl::PointXYZ>()), point_density_(0), edge_direction_(0) {}

  /**
   * @brief      Main function of the class. Refines the boundary defined by the original boundary
   *             poses and returns the refined poses.
   *
   * @details    This algorithm removes any NaN's from the original boundary poses, then:
   *             1) Finds all points within N points of each boundary pose using a nearest neighbor search.
   *             2) Removes all points that do not lie on the x-y plane of that pose with some allowed deviation.
   *             3) Determines which of the remaining points are "boundary points".
   *             4) Extracts those boundary point indices from the original point cloud.
   *             5) Determines which boundary point is closest to the original pose.
   *             6) Generates refined poses by moving the original pose orientations to the closest boundary point.
   *             7) Calculates if any of the new pose points make a large jump.
   *             8) Generates additional points that follow along the path of the actual boundary for the large jump.
   *             8) Determines which indices require addional poses and adds them to the refined poses.
   *              
   * @param[in]  original_boundary_poses  The original boundary poses
   * @param      refined_poses            The refined poses
   */
  void refineBoundary(const EigenPoseMatrix &original_boundary_poses, EigenPoseMatrix &refined_poses);

  /**
   * @brief      Sets the search radius for the boundary search.
   *
   * @param[in]  search_radius  The search radius
   */
  void setBoundarySearchRadius(float search_radius) { boundary_search_radius_ = search_radius; }

  /**
   * @brief      Gets the search radius for the boundary search.
   *
   * @return     The boundary search radius.
   */
  float getBoundarySearchRadius(void) { return boundary_search_radius_; }

  /**
   * @brief      Sets the number of neighbors to find for each boundary pose.
   *
   * @param[in]  number_of_neighbors  The number of neighbors
   */
  void setNumberOfNeighbors(int number_of_neighbors) { number_of_neighbors_ = number_of_neighbors; }

  /**
   * @brief      Gets the number of neighbors to find for each boundary pose.
   *
   * @return     The number of neighbors.
   */
  int getNumberOfNeighbors(void) { return number_of_neighbors_; }

  /**
   * @brief      Sets the debug display this requires the user to also set the visual cloud.
   *
   * @param[in]  debug_display  The debug display
   */
  void setDebugDisplay(bool debug_display) { if (debug_display) { debug_display_ = true; } }
  
  /**
   * @brief      Sets the visual cloud if debug display is enabled.
   *
   * @param[in]  colored_cloud_ptr  The colored cloud pointer
   */
  void setVisualCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud_ptr)
  {
    visual_cloud_->clear();
    for (const auto &pt : *colored_cloud_ptr) { visual_cloud_->push_back(pt); }
  }
  
private:
  /**
   * @brief      Determines if it an Eigen Matrix4f contains any NaNs
   *
   * @param[in]  matrix  The matrix
   *
   * @return     True if the matrix contains nans, False otherwise.
   */
  static bool containsNaNs(Eigen::Matrix4f matrix);

  /**
   * @brief      Iterates through a vector of poses and remove all NaN's
   *
   * @param[in]  original_boundary_poses  The original boundary poses
   * @param      boundary_poses_no_nan    The boundary poses without NaN's
   */
  static void removeNaNFromPoseTrajectory(const EigenPoseMatrix &original_boundary_poses,
                                          EigenPoseMatrix &boundary_poses_no_nan);

  /**
   * @brief      Iterates through a vector of boundary poses and creates a point cloud at each pose 
   *             of the nearest N points.
   *
   * @param[in]  input_cloud             The input cloud
   * @param[in]  boundary_poses          The boundary poses
   * @param[in]  number_of_neighbors     The number of neighbors
   * @param      boundary_pose_neighbor  Vector of point clouds containing neighbor points for every pose
   */
  static void nearestNNeighborSearch(const pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud,
                                     const EigenPoseMatrix &boundary_poses,
                                     const int &number_of_neighbors,
                                     PointCloudVector &boundary_pose_neighbor);

  /**
   * @brief      Iterates through a vector of boundary poses and creates a point cloud of each pose 
   *             by finding the nearest points within a search radius.
   *             Note: Not currently being used, kept for future reference.
   *
   * @param[in]  input_cloud              The input cloud
   * @param[in]  boundary_poses           The boundary poses
   * @param[in]  search_radius            The search radius
   * @param      boundary_pose_neighbors  Vector of point clouds containing neighbor points for every pose
   */
  static void nearestNeighborRadiusSearch(const pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud,
                                          const EigenPoseMatrix &boundary_poses,
                                          const float &search_radius,
                                          PointCloudVector &boundary_pose_neighbors);

  /**
   * @brief      Calculates the standard deviation given a vector of deviations.
   *
   * @param[in]  deviations  Vector of the deviations of a point from a plane.
   *
   * @return     The allowed deviation.
   */
  static float calculateAllowedDeviation(const std::vector<float> &deviations);

  /**
   * @brief      Calculates the result of a plane if the plane is: a*x + b*y + c*z = d
   *
   * @param[in]  x     x - point
   * @param[in]  y     y - point
   * @param[in]  z     z - point
   * @param[in]  a     a - coefficient
   * @param[in]  b     b - coefficient
   * @param[in]  c     c - coefficient
   * @param[in]  d     d - coefficient
   *
   * @return     a*x + b*y + c*z - d = return
   */
  static float calculatePlane(const float &x, const float &y, const float &z, 
                              const float &a, const float &b, const float &c, const float &d);

  /**
   * @brief      Given a vector of boundary poses and the point cloud at each pose,
   *             this function will refine the point clouds by removing the points that
   *             do not lie within the plane of the original pose within some tolerance.
   *             
   * @details    Given a point A = (Ax, Ay, Az), a normal vector at A, N = <Nx, Ny, Nz>,
   *             and a random point on the plane P = (Px, Py, Pz).
   *             
   *             To check if a point is in a plane:
   *             We know that the dot product between the vector AP and N should equal
   *             zero since they are orthogonal.
   *             
   *             dot(P-A, N) = 0 -> dot(P,N) - dot(A,N) = 0 -> dot(P,N) = dot(A,N)
   *             Since A and N are known, substitute values for P to check if it is on the plane.
   *             
   *             This function also calculates the deviation from the plane of all
   *             points in the point cloud. Then it calculates the standard deviation
   *             of the deviations to determine an allowed error.
   *             
   *             If the points in the point cloud fall within this allowed error, the 
   *             point is added into the refined cloud.                        
   *
   * @param[in]  boundary_poses                  The original boundary poses
   * @param[in]  boundary_pose_neighbor          The vector of point clouds within N neighbors
   * @param      refined_boundary_pose_neighbor  The vector of point clouds within plane
   */
  static void refineNeighborPoints(const EigenPoseMatrix &boundary_poses,
                                   const PointCloudVector &boundary_pose_neighbor,
                                   PointCloudVector &refined_boundary_pose_neighbor);

  /**
   * @brief      Calculates the normals for a point cloud.
   *
   * @param[in]  input_cloud  The input cloud
   * @param      normals      The point cloud of normals
   */
  static void computeNormals(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud, 
                             pcl::PointCloud<pcl::Normal> &normals);

  /**
   * @brief      Calculates the indicies of boundary points for each point cloud
   *             given a vector of point clouds.
   *
   * @param[in]  refined_cloud           The vector of refined point clouds at each pose
   * @param      boundary_search_radius  The search radius for PCL's boundary estimation
   * @param[in]  refined_boundary        The vector of boundary indices for each pose
   */
  static void computeBoundaryForRefinedCloud(const PointCloudVector &refined_cloud,
                                             const float boundary_search_radius,
                                             PointCloudBoundaryVector &refined_boundary);

  /**
   * @brief      Extracts the boundary points from a point cloud given a vector of clouds
   *             containing boundary point indices. Results in a vector of ordered point clouds
   *             of boundary points.
   *
   * @param[in]  refined_points_cloud  The vector of refined point clouds for each pose
   * @param[in]  boundary_cloud        The vector of boundary index clouds for each refined pose
   * @param      boundary_points       The vector of boundary point clouds for each pose
   */
  static void extractBoundaryPointsFromPointCloud(const PointCloudVector &refined_points_cloud,
                                                  const PointCloudBoundaryVector &boundary_cloud,
                                                  PointCloudVector &boundary_points);

  /**
   * @brief      Takes a point cloud of boundary points and orders them. 
   *             
   * @details    This function assumes that these boundary clouds contain points that
   *             form a shape that encloses something, is curved, and is closed.
   *             
   *             The algorithm starts with the default first point of the cloud, and calculates the two
   *             closest points. The first closest point will be itself, and the second closest point will
   *             be the closest point that is not itself. 
   *             
   *             That search point is added to the ordered cloud and removed from the search cloud.
   *             
   *             This is repeated until all the points are removed and we are left with
   *             a ordered point cloud.
   *             
   *             This is used in the future to generate trajectories along the boundaries.
   *             
   *             Note: This function also contains a visualization tool to check if the boundary
   *                   points were actually ordered correctly. This tool is disabled by default.
   *
   * @param[in]  point_cloud  The unordered point cloud
   *
   * @return     The ordered point cloud
   */
  static pcl::PointCloud<pcl::PointXYZ> defineOrderForPointCloud(const pcl::PointCloud<pcl::PointXYZ> &point_cloud);

  /**
   * @brief      Gets the distance between two points (TEMP FUNCTION).
   *
   * @param[in]  point_a  The point a
   * @param[in]  point_b  The point b
   *
   * @return     The distance between two points.
   */
  static float distanceBetweenTwoPoints(const pcl::PointXYZ &point_a, const pcl::PointXYZ &point_b);

  /**
   * @brief      Gets a RGB value given an intensity.
   *
   * @param[in]  intensity  The intensity
   *
   * @return     The rgb.
   */
  static std::vector<float> getRGB(float intensity);

  /**
   * @brief      Maps the intensity.
   *
   * @param[in]  x        The value you want to map
   * @param[in]  in_min   In minimum
   * @param[in]  in_max   In maximum
   * @param[in]  out_min  The out minimum
   * @param[in]  out_max  The out maximum
   *
   * @return     The mapped intensity.
   */
  static float mapIntensity(float x, float in_min, float in_max, float out_min, float out_max);

  /**
   * @brief      Calculates the closest point in boundary to pose.
   *
   * @param[in]  boundary_poses             The boundary poses
   * @param[in]  extracted_boundary_points  The extracted boundary points
   * @param      new_pose_points            The new pose points
   */
  static void calculateClosestPointInBoundaryToPose(const EigenPoseMatrix &boundary_poses,
                                                    const PointCloudVector &extracted_boundary_points,
                                                    PointVector &new_pose_points);

  /**
   * @brief      Creates a new pose by moving the position of the original
   *             pose to the closest boundary point while keeping the same orientation.
   *
   * @param[in]  boundary_poses       The original vector of boundary poses
   * @param[in]  new_boundary_points  The vector of closest the closest boundary point to the corresponding pose
   * @param      refined_poses        The refined poses
   */
  static void movePoseToNewPoint(const EigenPoseMatrix &boundary_poses,
                                 const PointVector &new_boundary_points,
                                 EigenPoseMatrix &refined_poses);

  /**
   * @brief      Calculates the index of the refined poses where there is a jump larger than some
   *             prefined threshold.
   *             
   *             NOTE: Debug statements have been left in the implementation since this is still in development.
   *             
   * @details    The outlier index is the first index of the jump. If the index is 100, the large gap is between poses
   *             100 and 101.          
   *
   * @param[in]  neighbor_new_pose_points  The neighbor new pose points
   * @param      outlier_index             The outlier index
   */
  static void calculateOutliersInNewPosePoints(const PointVector &neighbor_new_pose_points,
                                               std::map<int, int> &outlier_index);

  /**
   * @brief      Calculates the distance between two points given a point vector and the index of the points.
   *
   * @param[in]  point_vector  The point vector
   * @param[in]  index_1       The index 1
   * @param[in]  index_2       The index 2
   *
   * @return     The distance between two points.
   */
  static float distanceBetweenTwoPoints(const PointVector &point_vector,
                                        const int index_1, const int index_2);

  /**
   * @brief      Calculates the number of points to insert between outliers.
   *
   * @param[in]  distance_between_points  The distance between points
   * @param[in]  standard_deviation       The standard deviation
   *
   * @return     The number of points to insert.
   */
  static int calculateNumberOfPointsToInsert(const float &distance_between_points,
                                             const float &standard_deviation);

  /**
   * @brief      Calculates the maximum value in a vector of floats.
   *
   * @param      vec   The vector
   *
   * @return     The maximum value.
   */
  static float maxValueOfVector(std::vector<float> &vec);

  /**
   * @brief      Calculates the additional poses required to fill gaps.
   * 
   *             NOTE: Debug statements have been left in the implementation since this is still in development.
   *
   * @param[in]  boundary_points           The boundary points
   * @param[in]  neighbor_new_pose_points  The neighbor new pose points
   * @param[in]  outlier_index             The outlier index
   * @param      additional_poses          The additional poses
   */
  static void calculateAdditionalPosesRequiredToFillGaps(const PointCloudVector &boundary_points, 
                                                         const PointVector &neighbor_new_pose_points, 
                                                         const std::map<int, int> &outlier_index,
                                                         std::map<int, PointVector> &additional_poses);

  /**
   * @brief      Calculates a vector of points to fill the gab between the two poses.
   *             It will be left as an excercise to the reader to determine how this works.
   *
   *             NOTE: Debug statements have been left in the implementation since this is still in development.
   *
   * @param[in]  boundary_points           The boundary points
   * @param[in]  neighbor_new_pose_points  The neighbor new pose points
   * @param[in]  index                     The index
   * @param[in]  num_poses_required        The number poses required
   *
   * @return     A vector of points that fill the large gap between two poses.
   */
  static PointVector calculateClosestBoundaryPointToNextPose(const PointCloudVector &boundary_points, 
                                                            const PointVector &neighbor_new_pose_points, 
                                                            const int &index,
                                                            const int &num_poses_required);

  /**
   * @brief      Adds additional poses to refined poses. This results in the finalized vector of refined poses.
   * 
   *             NOTE: Debug statements have been left in the implementation since this is still in development.
   *
   * @param[in]  boundary_poses    The boundary poses
   * @param[in]  additional_poses  The additional poses
   * @param      refined_poses     The refined poses
   */
  static void addAdditionalPosesToRefinedPoses(const EigenPoseMatrix &boundary_poses,
                                               const std::map<int, PointVector> &additional_poses,
                                               EigenPoseMatrix &refined_poses);
  
  /**
   * @brief      Converts a point into an eigen pose matrix.
   *
   * @param[in]  boundary_poses  The boundary poses
   * @param[in]  points          The points
   * @param[in]  index           The index
   *
   * @return     An eigen pose matrix at that point.
   */
  static EigenPoseMatrix convertPointsToEigenMatrix(const EigenPoseMatrix &boundary_poses,
                                                    const PointVector &points,
                                                    const int &index);

  /**
   * @brief      Gets the point density.
   * 
   *             Some function Dr. Chris Lewis wrote, leaving it in here in case we
   *             need it in the future.
   *             
   * @return     The point density.
   */
  float getPointDensity(void);

  /**
   * @brief      Keyboard callback for debug display.
   *
   * @param[in]  event                    The event
   * @param      debug_display_data_void  The debug display data void
   */
  static void keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event,
                                    void* debug_display_data_void);

  /**
   * @brief      Displays a pcl visualizer showning every step of the edge refinement algorithm.
   *
   * @param[in]  boundary_poses                  The boundary poses
   * @param[in]  boundary_pose_neighbor          The boundary pose neighbor
   * @param[in]  refined_boundary_pose_neighbor  The refined boundary pose neighbor
   * @param[in]  neighbor_boundary_points        The neighbor boundary points
   * @param[in]  new_pose_points                 The new pose points
   * @param[in]  refined_poses                   The refined poses
   * @param[in]  additional_poses                The additional poses
   */
  void debugDisplay(const EigenPoseMatrix &boundary_poses,
                    const PointCloudVector &boundary_pose_neighbor,
                    const PointCloudVector &refined_boundary_pose_neighbor,
                    const PointCloudVector &neighbor_boundary_points,
                    const PointVector &new_pose_points,
                    const EigenPoseMatrix &refined_poses,
                    const std::map<int, PointVector> &additional_poses);

private:
  pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_;
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_;

  bool debug_display_;
  float point_density_;
  double radius_;
  double sradius_;
  int edge_direction_;

  float boundary_search_radius_;
  int number_of_neighbors_;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr visual_cloud_;

  std::size_t current_pose_index_;
  std::size_t num_poses_;
};

} // namespace godel_scan_tools
#endif // EDGE_REFINEMENT_H