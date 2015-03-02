#include <ros/ros.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

typedef pcl::PointCloud<pcl::PointXYZ> point_cloud_t;

void generateFakeData(point_cloud_t& cloud)
{
  static double x = 0.0;
  static double y = 0.0;
  static double z = 0.0;
  cloud.points.push_back(pcl::PointXYZ(x, y, z));
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_scan_publisher");

  ros::NodeHandle nh;

  ros::Publisher pub = nh.advertise<point_cloud_t>("scan_server/scan", 100);

  ros::Rate pub_rate(1);
  while (ros::ok())
  {
    point_cloud_t::Ptr cloud_ptr(new point_cloud_t);
    generateFakeData(*cloud_ptr);
    pub.publish(cloud_ptr);
    pub_rate.sleep();
  }
}