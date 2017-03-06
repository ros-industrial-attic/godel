#include <segmentation/surface_segmentation.h>
#include <ros/ros.h>

/*
 * A stand-alone node for testing Godel's "SurfaceSegmentation" class w/o the
 * infrastructure required for the entire system.
 */

static pcl::PointCloud<pcl::PointXYZRGB>::Ptr
computeBoundaryCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
  const static double SEGMENTATION_SEARCH_RADIUS = 0.03; // 3cm

  SurfaceSegmentation segmenter (cloud);
  segmenter.setSearchRadius(SEGMENTATION_SEARCH_RADIUS);

  pcl::PointCloud<pcl::Boundary>::Ptr boundary_ptr (new pcl::PointCloud<pcl::Boundary>());
  segmenter.getBoundaryCloud(boundary_ptr);

  // Return data structures
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr boundary_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>());
  pcl::IndicesPtr boundary_idx(new std::vector<int>());

  // Populate color cloud
  int k=0;
  for(const auto& pt : boundary_ptr->points)
  {
    if(pt.boundary_point)
    {
      boundary_cloud_ptr->points.push_back(cloud->points[k]);
      boundary_idx->push_back(k);
    }
    k++;
  }

  boundary_cloud_ptr->width = 1;
  boundary_cloud_ptr->height = boundary_cloud_ptr->points.size();

  return boundary_cloud_ptr;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "boundary_test_node");
  ros::NodeHandle pnh ("~");

  std::string filename;
  if (!pnh.getParam("filename", filename))
  {
    ROS_ERROR("Node requires user to set private parameter 'filename'");
    return 1;
  }

  auto cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();

  if (pcl::io::loadPCDFile(filename, *cloud) < 0)
  {
    ROS_ERROR("Could not load cloud file: %s", filename.c_str());
    return 2;
  }

  ROS_INFO("Starting boundary extraction routine...");
  auto start_tm = ros::Time::now();
  auto boundary_cloud = computeBoundaryCloud(cloud);
  auto finish_tm = ros::Time::now();
  ROS_INFO("Boundary extract completed after %f seconds.", (finish_tm - start_tm).toSec());

  pcl::io::savePCDFile("boundary.pcd", *boundary_cloud);
  return 0;
}
