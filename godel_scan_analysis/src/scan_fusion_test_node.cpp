#include <ros/ros.h>

#include "godel_scan_analysis/profile_scan_fusion.h"
#include "godel_scan_analysis/profile_scan_aggregator.h"

#include <std_srvs/Trigger.h>

void scanCallback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& profile,
                  godel_scan_server::ProfileScanAggregator& agg)
{
  agg.addProfile(*profile);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "scan_fusion_test_node");
  ros::NodeHandle nh, pnh ("~");

  // Read params
  std::string base_frame = pnh.param<std::string>("base_frame", "base_link");
  std::string scan_frame = pnh.param<std::string>("scan_frame", "keyence_sensor_optical_frame");

  godel_scan_analysis::ProfileFusionParameters params;
  params.voxel_leaf_size = pnh.param<double>("voxel_size", 0.0005); // 0.5 mm
  params.min_points_per_voxel = pnh.param<int>("min_points_per_voxel", 1);

  // Create aggregator
  godel_scan_server::ProfileScanAggregator aggregator (base_frame, scan_frame);

  // Create services to clear
  ros::ServiceServer reset_service =
      nh.advertiseService<std_srvs::TriggerRequest, std_srvs::TriggerResponse>("reset_scans",
        [&aggregator] (const std_srvs::TriggerRequest&, std_srvs::TriggerResponse&) {
           aggregator.clear();
           return true;
        }
  );

  // Create subscribers
  ros::Subscriber profile_sub = nh.subscribe<pcl::PointCloud<pcl::PointXYZ>>("profiles", 0, boost::bind(scanCallback, _1, boost::ref(aggregator)));

  ros::spin();
}
