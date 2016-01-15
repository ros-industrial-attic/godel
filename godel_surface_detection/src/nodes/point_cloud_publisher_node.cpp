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

#include <ros/ros.h>
#include <ros/package.h>
#include <boost/filesystem.hpp>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
#include <pcl/console/parse.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>

using namespace boost::filesystem;

const float DEFAULT_NOISE_RANGE = 0.005f;
const float DEFAULT_PUBLISH_RATE = 0.5f;
const std::string CLOUD_TRANSFORM_PARAM = "cloud_transform";
const std::string DEFAULT_FRAME_ID = "world_frame";
const std::string POINT_CLOUD_TOPIC = "sensor_point_cloud";
const std::string HELP_TEXT = "\n-h Help information\n-f <file name>\n"
                              "-n <noise range value (m)>\n-r <publish rate (sec)>\n-i <frame id> ";

typedef pcl::PointCloud<pcl::PointXYZRGBA> CloudRGBA;
typedef pcl::PointCloud<pcl::PointXYZRGB> CloudRGB;

bool read_transform(ros::NodeHandle& nh, std::string param_name, tf::Transform& t)
{
  XmlRpc::XmlRpcValue val;
  double x, y, z, rx, ry, rz;
  bool succeeded = nh.getParam(param_name, val);
  if (succeeded)
  {
    x = val["x"].getType() == val.TypeDouble ? static_cast<double>(val["x"]) : 0;
    y = val["y"].getType() == val.TypeDouble ? static_cast<double>(val["y"]) : 0;
    z = val["z"].getType() == val.TypeDouble ? static_cast<double>(val["z"]) : 0;
    rx = val["rx"].getType() == val.TypeDouble ? static_cast<double>(val["rx"]) : 0;
    ry = val["ry"].getType() == val.TypeDouble ? static_cast<double>(val["ry"]) : 0;
    rz = val["rz"].getType() == val.TypeDouble ? static_cast<double>(val["rz"]) : 0;

    t.setOrigin(tf::Vector3(x, y, z));
    t.getBasis().setRPY(rx, ry, rz);
  }

  return succeeded;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "point_cloud_publisher_node");

  // point cloud publisher
  ros::NodeHandle nh;
  ros::NodeHandle ph("~"); // private handle for parameter reading
  ros::Publisher point_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>(POINT_CLOUD_TOPIC, 1);

  // arguments
  path file_path;
  float noise_range = DEFAULT_NOISE_RANGE;
  float rate = DEFAULT_PUBLISH_RATE;
  std::string frame_id = DEFAULT_FRAME_ID;

  // point clouds
  CloudRGB::Ptr cloud_ptr(new CloudRGB());
  CloudRGB::Ptr noise_cloud_ptr(new CloudRGB());

  if (argc > 1)
  {
    if (pcl::console::find_switch(argc, argv, "-h"))
    {
      ROS_INFO_STREAM(HELP_TEXT);
      return 0;
    }

    if (pcl::console::find_switch(argc, argv, "-f"))
    {
      // parsing arguments
      std::string fpath;
      pcl::console::parse(argc, argv, "-f", fpath);
      file_path = path(ros::package::getPath("godel_surface_detection") + "/" + fpath);

      if (exists(file_path))
      {
        ROS_INFO_STREAM("Found file: " << file_path.c_str());
      }
      else
      {
        ROS_ERROR_STREAM("Must supply valid pcd file name that exists in the package directory");
        return 0;
      }
    }
    else
    {
      ROS_ERROR_STREAM("-f argument not found.  Must supply pcd file");
      return 0;
    }

    if (pcl::console::find_switch(argc, argv, "-n"))
    {
      pcl::console::parse(argc, argv, "-n", noise_range);
    }

    if (pcl::console::find_switch(argc, argv, "-r"))
    {
      pcl::console::parse(argc, argv, "-r", rate);
    }

    if (pcl::console::find_switch(argc, argv, "-i"))
    {
      pcl::console::parse(argc, argv, "-i", frame_id);
    }

    ROS_INFO_STREAM("Using noise " << noise_range);
    ROS_INFO_STREAM("Using rate " << rate);
    ROS_INFO_STREAM("Using frame_id " << frame_id);
  }
  else
  {
    ROS_ERROR_STREAM("Must supply valid pcd file name that exists in the package directory");
    ROS_ERROR_STREAM(HELP_TEXT);
    return 0;
  }

  // reading pcd file
  if (pcl::io::loadPCDFile(std::string(file_path.c_str()), *cloud_ptr) == -1)
  {
    ROS_ERROR_STREAM("Failed to read pcd file");
    return 0;
  }
  else
  {
    ROS_INFO_STREAM("Successfully read pcd file with " << cloud_ptr->points.size() << " points");
  }

  // publishing
  ros::Duration loop_time(rate);
  tf::Transform cloud_transform = tf::Transform::getIdentity();
  srand(time(NULL));
  float noise;

  ROS_INFO_STREAM("Publishing cloud ");
  while (ros::ok())
  {

    // adding noise
    pcl::copyPointCloud(*cloud_ptr, *noise_cloud_ptr);
    for (int i = 0; i < noise_cloud_ptr->points.size(); i++)
    {
      noise = float(rand()) / float(RAND_MAX); // <0,1>
      noise = -noise_range * 0.5f + noise * noise_range;
      pcl::PointXYZRGB& p = noise_cloud_ptr->points[i];
      noise_cloud_ptr->points[i].x = p.x + noise;
      noise_cloud_ptr->points[i].y = p.y + noise;
      noise_cloud_ptr->points[i].z = p.z + noise;
    }

    // transforming point_cloud
    if (read_transform(ph, CLOUD_TRANSFORM_PARAM, cloud_transform))
    {
      Eigen::Affine3d eigen_transform;
      tf::poseTFToEigen(cloud_transform, eigen_transform);
      pcl::transformPointCloud(*noise_cloud_ptr, *noise_cloud_ptr,
                               (Eigen::Affine3f)(eigen_transform));
    }

    // convert to msg
    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(*noise_cloud_ptr, msg);

    msg.header.frame_id = msg.header.frame_id.empty() ? frame_id : msg.header.frame_id;
    point_cloud_pub.publish(msg);

    loop_time.sleep();
  }

  return 0;
}
