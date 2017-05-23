#include "godel_qa_server/qa_server.h"

godel_qa_server::QAJob::QAJob(const cat_laser_scan_qa::TorchCutQAParameters &params)
  : params_{params}
{

}

cat_laser_scan_qa::TorchCutQAResult godel_qa_server::QAJob::addNewScan(const pcl::PointCloud<pcl::PointXYZ> &new_scan)
{
  auto result = cat_laser_scan_qa::runQualityAssurance(new_scan.makeShared(), params_);
  iters_.emplace_back( Iteration{new_scan, result} );
  return result;
}

void godel_qa_server::QAServer::createNewJob(godel_qa_server::QAServer::key_type surface_id)
{
  auto r = active_jobs_.insert({surface_id, {params_}});
  if (!r.second)
  {
    ROS_WARN("Surface QA Job for surface with id = %d already in server.", surface_id);
    throw std::runtime_error("QAServer: Job for surface already present");
  }
}

void godel_qa_server::QAServer::clear()
{
  active_jobs_.clear();
}

boost::optional<const godel_qa_server::QAJob &> godel_qa_server::QAServer::lookup(key_type surface_id) const
{
  const auto it = active_jobs_.find(surface_id);
  if (it != active_jobs_.end())
    return it->second;
  else
    return {};
}
