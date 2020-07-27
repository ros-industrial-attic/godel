#ifndef GODEL_QA_SERVER_H
#define GODEL_QA_SERVER_H

#include <cat_laser_scan_qa/torch_cut_qa.h>
#include <map>
#include <boost/optional.hpp>

namespace godel_qa_server
{

/**
 * @brief A 'QAJob' is a record of the various laser scans and QA info that are collected
 * for a given part as we work on it. One could use this info to, for example, show how a
 * surface was sanding into spec as time goes on.
 */
class QAJob
{
public:
  struct Iteration
  {
    pcl::PointCloud<pcl::PointXYZ> scan;
    cat_laser_scan_qa::TorchCutQAResult qa_result;
  };

  QAJob(const cat_laser_scan_qa::TorchCutQAParameters& params);

  cat_laser_scan_qa::TorchCutQAResult addNewScan(const pcl::PointCloud<pcl::PointXYZ>& new_scan);

  const std::vector<Iteration>& iterations() const { return iters_; }

private:
  std::vector<Iteration> iters_;
  cat_laser_scan_qa::TorchCutQAParameters params_;
};

/**
 * @brief The QAServer that keeps track of the history of QA passes after processing steps.
 *
 * For the moment this is meant to work with surface finishing of flat parts, and does the following:
 * 1. First one creates a new "QA job" - this job will keep track of laser scans and QA info as we iterate on a part
 * 2. Add a new laser scan. The server will automatically run a QA pass, record it, and also return it
 * 3. (TBD?) Implement business logic (e.g. we're done after 10 passes)?
 */
class QAServer
{
public:
  using key_type = int;

  void setParams(const cat_laser_scan_qa::TorchCutQAParameters& params)
  {
    params_ = params;
  }

  void createNewJob(key_type surface_id);

  void clear();

  boost::optional<const QAJob&> lookup(key_type surface_id) const;

  boost::optional<QAJob&> lookup(key_type surface_id);

  using JobConstIter = std::map<key_type, QAJob>::const_iterator;

  JobConstIter begin() const { return active_jobs_.begin(); }

  JobConstIter end() const { return active_jobs_.end(); }


private:
  std::map<key_type, QAJob> active_jobs_;
  cat_laser_scan_qa::TorchCutQAParameters params_;
};

}

#endif // GODEL_QA_SERVER_H
