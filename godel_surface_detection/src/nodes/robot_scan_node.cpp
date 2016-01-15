/*
        Copyright Apr 15, 2014 Southwest Research Institute

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

#include <godel_surface_detection/scan/robot_scan.h>
#include <godel_surface_detection/detection/surface_detection.h>
#include <godel_surface_detection/interactive/interactive_surface_server.h>
#include <pcl/console/parse.h>

using namespace godel_surface_detection;

// constants
const std::string HELP_TEXT = "\n-h Help information\n-m <move mode only (0|1)>\n";
const std::string SEGMENTS_CLOUD_TOPIC = "segments_cloud";
const std::string DISPLAY_TRAJECTORY_TOPIC = "scan_trajectory";
const std::string HOME_POSITION = "home_asus";

// robot scan instance
godel_surface_detection::scan::RobotScan RobotScan;

// surface detection instance
godel_surface_detection::detection::SurfaceDetection SurfDetect;

// marker server instance
godel_surface_detection::interactive::InteractiveSurfaceServer SurfServer;

// defaults
const bool DEFAULT_MOVE_ONLY_MODE = true;

struct CmdArgs
{
  bool move_mode_only_;
};

// functions

bool parse_arguments(int argc, char** argv, CmdArgs& args)
{
  // filling defaults
  args.move_mode_only_ = true;

  if (argc > 1)
  {
    if (pcl::console::find_switch(argc, argv, "-h"))
    {
      ROS_INFO_STREAM(HELP_TEXT);
      return false;
    }

    if (pcl::console::find_switch(argc, argv, "-m") &&
        pcl::console::parse(argc, argv, "-m", args.move_mode_only_) >= 0)
    {
      ROS_INFO_STREAM("arg 'move mode only (-m)': " << (args.move_mode_only_ ? "true" : "false"));
    }
  }

  return true;
}

bool init()
{
  // load parameters for all objects
  bool succeeded = SurfDetect.load_parameters("~/surface_detection") &&
                   SurfServer.load_parameters("") && RobotScan.load_parameters("~/robot_scan");

  if (succeeded)
  {
    ROS_INFO_STREAM("Parameters loaded");
    ROS_INFO_STREAM("Robot Scan Parameters: \n" << RobotScan.params_);

    if (SurfDetect.init() && SurfServer.init() && RobotScan.init())
    {
      // starting server
      SurfServer.run();

      // adding callbacks to robot scan
      scan::RobotScan::ScanCallback cb =
          boost::bind(&detection::SurfaceDetection::add_cloud, &SurfDetect, _1);
      RobotScan.add_scan_callback(cb);
    }
  }
  else
  {
    ROS_ERROR_STREAM("Parameters did not load");
  }

  return succeeded;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "RobotScan_node");
  ros::AsyncSpinner spinner(2);
  spinner.start();
  ros::NodeHandle nh;

  // publishers
  ros::Publisher point_cloud_publisher =
      nh.advertise<sensor_msgs::PointCloud2>(SEGMENTS_CLOUD_TOPIC, 1);

  // parsing arguments
  CmdArgs args;
  if (!parse_arguments(argc, argv, args))
  {
    return 0;
  }

  // initializing all objects
  if (init())
  {

    // moving to home position
    RobotScan.get_move_group()->setNamedTarget(HOME_POSITION);
    if (!RobotScan.get_move_group()->move())
    {
      ROS_ERROR_STREAM("Robot failed to move home");
      return 0;
    }

    // publish poses
    RobotScan.publish_scan_poses(DISPLAY_TRAJECTORY_TOPIC);

    // moving through each pose (do not scan)
    int reached_points = RobotScan.scan(args.move_mode_only_);

    ROS_INFO_STREAM("Scan points reached: " << reached_points);

    // moving back to home position
    RobotScan.get_move_group()->setNamedTarget(HOME_POSITION);
    if (!RobotScan.get_move_group()->move())
    {
      ROS_ERROR_STREAM("Robot failed to move home");
      return 0;
    }

    // finding surfaces
    if (SurfDetect.find_surfaces())
    {
      ROS_INFO_STREAM("Publishing segments visuals");
      sensor_msgs::PointCloud2 cloud_msg;
      std::vector<pcl::PolygonMesh> meshes;
      SurfDetect.get_meshes(meshes);
      SurfDetect.get_region_colored_cloud(cloud_msg);

      // adding markers to server
      for (int i = 0; i < meshes.size(); i++)
      {
        SurfServer.add_surface(meshes[i]);
      }

      ros::Duration loop_rate(0.5f);
      while (ros::ok())
      {
        point_cloud_publisher.publish(cloud_msg);
        ros::spinOnce();
        loop_rate.sleep();
      }

      point_cloud_publisher.shutdown();
    }
    else
    {
      ROS_WARN_STREAM("No surfaces were found, exiting");
    }
  }
  else
  {
    ROS_ERROR_STREAM("Robot scan object did not initialized property");
  }

  spinner.stop();

  return 0;
}
