/*
        Copyright Feb 10, 2014 Southwest Research Institute

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

#include <godel_surface_detection/interactive/interactive_surface_server.h>
#include <tf/transform_datatypes.h>
#include <tf/tf.h>
#include <sstream>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl_ros/transforms.h>

namespace godel_surface_detection
{
namespace interactive
{

InteractiveSurfaceServer::InteractiveSurfaceServer()
    : marker_name_(defaults::MARKER_SERVER_NAME), marker_description_(defaults::MARKER_DESCRIPTION),
      arrow_distance_(defaults::ARROW_DISTANCE),
      arrow_head_diameter_(defaults::ARROW_HEAD_DIAMETER),
      arrow_head_length_(defaults::ARROW_HEAD_LENGTH), arrow_length_(defaults::ARROW_LENGTH),
      arrow_shaft_diameter_(defaults::ARROW_SHAFT_DIAMETER)
{
  // TODO Auto-generated constructor stub
}

InteractiveSurfaceServer::~InteractiveSurfaceServer()
{
  // TODO Auto-generated destructor stub
}

void InteractiveSurfaceServer::mesh_to_marker(const pcl::PolygonMesh& mesh,
                                              visualization_msgs::Marker& marker)
{
  // color value ranges
  static const double color_val_min = 0.5f;
  static const double color_val_max = 1.0f;
  std_msgs::ColorRGBA color;
  color.a = 1;

  // set marker properties
  tf::poseTFToMsg(tf::Transform::getIdentity(), marker.pose);
  marker.scale.x = marker.scale.y = marker.scale.z = 1;
  marker.type = marker.TRIANGLE_LIST;
  marker.action = marker.ADD;
  marker.header.frame_id = mesh.header.frame_id;

  // create color
  color.r = color_val_min +
            (static_cast<double>(rand()) / static_cast<double>(RAND_MAX)) *
                (color_val_max - color_val_min);
  color.g = color_val_min +
            (static_cast<double>(rand()) / static_cast<double>(RAND_MAX)) *
                (color_val_max - color_val_min);
  color.b = color_val_min +
            (static_cast<double>(rand()) / static_cast<double>(RAND_MAX)) *
                (color_val_max - color_val_min);
  color.a = 1.0f;
  marker.color = color;

  // filling points
  pcl::PointCloud<pcl::PointXYZ> points;
  pcl::fromPCLPointCloud2(mesh.cloud, points);
  for (int i = 0; i < mesh.polygons.size(); i++)
  {
    const pcl::Vertices& v = mesh.polygons[i];
    for (int j = 0; j < v.vertices.size(); j++)
    {
      uint32_t index = v.vertices[j];
      geometry_msgs::Point p;
      p.x = points.points[index].x;
      p.y = points.points[index].y;
      p.z = points.points[index].z;
      marker.points.push_back(p);
    }
  }
}

void InteractiveSurfaceServer::marker_to_mesh(const visualization_msgs::Marker& marker,
                                              pcl::PolygonMesh& mesh)
{
  pcl::PointCloud<pcl::PointXYZ> points;
  for (int i = 0; i < marker.points.size(); i += 3)
  {
    pcl::Vertices v;
    for (int j = 0; j < 3; j++)
    {
      v.vertices.push_back(i + j);

      pcl::PointXYZ p;
      p.x = marker.points[i + j].x;
      p.y = marker.points[i + j].y;
      p.z = marker.points[i + j].z;
      points.points.push_back(p);
    }

    mesh.polygons.push_back(v);
  }

  pcl::toPCLPointCloud2(points, mesh.cloud);
  mesh.header.frame_id = marker.header.frame_id;
}

bool InteractiveSurfaceServer::init()
{

  srand(time(NULL));
  return true;
}

void InteractiveSurfaceServer::run()
{
  marker_server_ptr_ = interactive_markers::InteractiveMarkerServerPtr(
      new interactive_markers::InteractiveMarkerServer(defaults::MARKER_SERVER_NAME, "", false));
  // create callbacks
  button_callback_ = interactive_markers::InteractiveMarkerServer::FeedbackCallback(
      boost::bind(&InteractiveSurfaceServer::button_marker_callback, this, _1));

  menu_callback_ = interactive_markers::InteractiveMarkerServer::FeedbackCallback(
      boost::bind(&InteractiveSurfaceServer::menu_marker_callback, this, _1));

  // setup menu handler
  select_entry_id_ = menu_handler_.insert("Select", menu_callback_);
  unselect_entry_id_ = menu_handler_.insert("Unselect", menu_callback_);
  interactive_markers::MenuHandler::EntryHandle submenu_handle =
      menu_handler_.insert("More Options");
  select_all_entry_id_ = menu_handler_.insert(submenu_handle, "Select All", menu_callback_);
  clear_all_entry_id_ = menu_handler_.insert(submenu_handle, "Clear Selections", menu_callback_);
  hide_entry_id_ = menu_handler_.insert(submenu_handle, "Hide", menu_callback_);
  show_all_entry_id_ = menu_handler_.insert(submenu_handle, "Show All", menu_callback_);

  marker_server_ptr_->applyChanges();
}

void InteractiveSurfaceServer::stop()
{
  marker_server_ptr_.reset();
  surface_selection_map_.clear();
}

void InteractiveSurfaceServer::set_selection_flag(std::string marker_name, bool selected)
{

  visualization_msgs::InteractiveMarker int_marker;
  if (surface_selection_map_.count(marker_name) > 0 &&
      marker_server_ptr_->get(marker_name, int_marker))
  {
    surface_selection_map_[marker_name] = selected;
    int_marker.controls[1].markers[0].type =
        selected ? visualization_msgs::Marker::ARROW : visualization_msgs::Marker::SPHERE;
    int_marker.controls[1].markers[0].scale.x = selected ? arrow_shaft_diameter_ : 0.0001f;
    int_marker.controls[1].markers[0].scale.y = selected ? arrow_head_diameter_ : 0.0001f;
    int_marker.controls[1].markers[0].scale.z = selected ? arrow_head_length_ : 0.0001f;

    marker_server_ptr_->insert(int_marker);
    invoke_callbacks();
    marker_server_ptr_->applyChanges();
  }
}

void InteractiveSurfaceServer::show(std::string marker_name, bool show)
{
  visualization_msgs::InteractiveMarker int_marker;
  if (surface_selection_map_.count(marker_name) > 0 &&
      marker_server_ptr_->get(marker_name, int_marker))
  {

    int_marker.controls[0].markers[0].scale.x = show ? 1 : 0.0001f;
    int_marker.controls[0].markers[0].scale.y = show ? 1 : 0.0001f;
    int_marker.controls[0].markers[0].scale.z = show ? 1 : 0.0001f;
    marker_server_ptr_->insert(int_marker);
    set_selection_flag(marker_name, show && surface_selection_map_[marker_name]);
  }
}

void InteractiveSurfaceServer::select_all(bool select)
{
  typedef std::map<std::string, bool>::iterator SelectionIterator;
  for (SelectionIterator i = surface_selection_map_.begin(); i != surface_selection_map_.end(); i++)
  {
    set_selection_flag(i->first, select);
  }
}

void InteractiveSurfaceServer::show_all(bool show_surf)
{
  typedef std::map<std::string, bool>::iterator SelectionIterator;
  for (SelectionIterator i = surface_selection_map_.begin(); i != surface_selection_map_.end(); i++)
  {
    show(i->first, show_surf);
  }
}

void InteractiveSurfaceServer::invoke_callbacks()
{
  for (unsigned int i = 0; i < selection_callbacks_.size(); i++)
  {
    selection_callbacks_[i]();
  }
}

void InteractiveSurfaceServer::get_selected_list(std::vector<std::string>& list)
{
  std::map<std::string, bool>::iterator i;
  for (i = surface_selection_map_.begin(); i != surface_selection_map_.end(); i++)
  {
    if (i->second)
    {
      list.push_back(i->first);
    }
  }
}

void InteractiveSurfaceServer::get_selected_surfaces(visualization_msgs::MarkerArray& surfaces)
{
  std::map<std::string, bool>::iterator i;
  for (i = surface_selection_map_.begin(); i != surface_selection_map_.end(); i++)
  {
    if (i->second)
    {
      visualization_msgs::InteractiveMarker int_marker;
      const std::string& marker_name = i->first;
      marker_server_ptr_->get(marker_name, int_marker);
      surfaces.markers.push_back(int_marker.controls[0].markers[0]);
    }
  }
}

void InteractiveSurfaceServer::get_selected_surfaces(std::vector<pcl::PolygonMesh>& meshes)
{
  std::map<std::string, bool>::iterator i;
  for (i = surface_selection_map_.begin(); i != surface_selection_map_.end(); i++)
  {
    if (i->second)
    {
      meshes.push_back(meshes_map_[i->first]);
    }
  }
}

void InteractiveSurfaceServer::toggle_selection_flag(std::string marker_name)
{
  if (surface_selection_map_.count(marker_name) > 0)
  {
    set_selection_flag(marker_name, !surface_selection_map_[marker_name]);
  }
}

void InteractiveSurfaceServer::button_marker_callback(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)
{
  switch (feedback->event_type)
  {
  case visualization_msgs::InteractiveMarkerFeedback::BUTTON_CLICK:
    // ROS_INFO_STREAM("marker "<<feedback->marker_name <<" button control was clicked");
    toggle_selection_flag(feedback->marker_name);
    break;

  case visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT:
    break;
  }
}

void InteractiveSurfaceServer::menu_marker_callback(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)
{

  switch (feedback->event_type)
  {
  case visualization_msgs::InteractiveMarkerFeedback::BUTTON_CLICK:
    //		ROS_INFO_STREAM("marker "<<feedback->marker_name <<" button control was clicked");
    toggle_selection_flag(feedback->marker_name);
    break;

  case visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT:
    //		ROS_INFO_STREAM("marker: "<<feedback->marker_name <<", entry_id: "<<
    //feedback->menu_entry_id
    //				<<", menu control was clicked");

    if (feedback->menu_entry_id == select_entry_id_)
    {
      set_selection_flag(feedback->marker_name, true);
      return;
    }
    else if (feedback->menu_entry_id == unselect_entry_id_)
    {
      set_selection_flag(feedback->marker_name, false);
      return;
    }
    else if (feedback->menu_entry_id == clear_all_entry_id_)
    {

      select_all(false);
      return;
    }
    else if (feedback->menu_entry_id == select_all_entry_id_)
    {
      select_all(true);
      return;
    }
    else if (feedback->menu_entry_id == hide_entry_id_)
    {
      // ROS_WARN_STREAM("'Hide' menu option has not been implemented yet");

      show(feedback->marker_name, false);
      return;
    }
    else if (feedback->menu_entry_id == show_all_entry_id_)
    {
      // ROS_WARN_STREAM("'Show All' menu option has not been implemented yet");
      show_all(true);
      marker_server_ptr_->applyChanges();
      return;
    }

    break;
  }
}

void InteractiveSurfaceServer::create_arrow_marker(const visualization_msgs::Marker& surface_marker,
                                                   visualization_msgs::Marker& arrow_marker)
{
  // create temporary point cloud
  pcl::PointCloud<pcl::PointXYZ> surface;
  surface.width = surface_marker.points.size();
  surface.height = 1;
  surface.points.resize(surface_marker.points.size());
  for (int i = 0; i < surface_marker.points.size(); i++)
  {
    surface.points[i].x = surface_marker.points[i].x;
    surface.points[i].y = surface_marker.points[i].y;
    surface.points[i].z = surface_marker.points[i].z;
  }

  // finding bouding box bounds
  pcl::PointXYZ min, max;
  pcl::getMinMax3D(surface, min, max);

  // create arrow
  arrow_marker.type = arrow_marker.ARROW;
  arrow_marker.scale.x = arrow_shaft_diameter_;
  arrow_marker.scale.y = arrow_head_diameter_;
  arrow_marker.scale.z = arrow_head_length_;
  arrow_marker.color.r = 0;
  arrow_marker.color.g = arrow_marker.color.b = arrow_marker.color.a = 1;
  arrow_marker.points.resize(2);

  // start point
  arrow_marker.points[1].x = 0.5f * (min.x + max.x);
  arrow_marker.points[1].y = 0.5f * (min.y + max.y);
  arrow_marker.points[1].z = max.z + arrow_distance_;

  // end point
  arrow_marker.points[0].x = arrow_marker.points[1].x;
  arrow_marker.points[0].y = arrow_marker.points[1].y;
  arrow_marker.points[0].z = arrow_marker.points[1].z + arrow_length_;
}

void InteractiveSurfaceServer::add_surface(const pcl::PolygonMesh& mesh,
                                           const geometry_msgs::Pose& pose)
{
  // convert polygon mesh to marker
  visualization_msgs::Marker marker;
  mesh_to_marker(mesh, marker);
  marker.id = meshes_map_.size();
  marker.color.a = defaults::MARKER_ALPHA;

  // create interactive marker
  std::stringstream ss;
  ss << marker_name_ << "_" << surface_selection_map_.size() + 1;
  visualization_msgs::InteractiveMarker int_marker;
  int_marker.name = ss.str();
  int_marker.pose = pose;

  // create button control
  visualization_msgs::InteractiveMarkerControl button_control;
  button_control.interaction_mode = button_control.BUTTON;
  button_control.markers.push_back(marker);
  button_control.name = "button_" + int_marker.name;
  button_control.always_visible = true;

  // create seletected arrow marker
  visualization_msgs::Marker arrow_marker;
  create_arrow_marker(marker, arrow_marker);
  visualization_msgs::InteractiveMarkerControl selected_arrow;
  selected_arrow.interaction_mode = selected_arrow.FIXED;
  selected_arrow.markers.push_back(arrow_marker);
  selected_arrow.name = "selected_" + int_marker.name;
  selected_arrow.always_visible = true;

  // fill interactive marker
  int_marker.controls.push_back(button_control);
  int_marker.controls.push_back(selected_arrow);
  int_marker.scale = 1;
  int_marker.header.frame_id = marker.header.frame_id;
  int_marker.description = marker_description_;

  // add marker to server
  marker_server_ptr_->insert(int_marker, button_callback_);
  menu_handler_.apply(*marker_server_ptr_, int_marker.name);

  // save mesh
  surface_selection_map_.insert(std::make_pair(int_marker.name, false));
  meshes_map_.insert(std::make_pair(int_marker.name, mesh));
  set_selection_flag(int_marker.name, false);

  // apply changes
  marker_server_ptr_->applyChanges();
}

void InteractiveSurfaceServer::remove_all_surfaces()
{
  surface_selection_map_.clear();
  marker_server_ptr_->clear();
  meshes_map_.clear();
  invoke_callbacks();
  marker_server_ptr_->applyChanges();
}

void InteractiveSurfaceServer::add_surface(const pcl::PolygonMesh& mesh)
{
  geometry_msgs::Pose pose;
  tf::poseTFToMsg(tf::Transform::getIdentity(), pose);
  add_surface(mesh, pose);
}

void InteractiveSurfaceServer::add_surface(const visualization_msgs::Marker& marker)
{
  pcl::PolygonMesh mesh;
  marker_to_mesh(marker, mesh);
  add_surface(mesh);
}

void InteractiveSurfaceServer::add_surface(const visualization_msgs::Marker& marker,
                                           const geometry_msgs::Pose& pose)
{
  pcl::PolygonMesh mesh;
  marker_to_mesh(marker, mesh);
  add_surface(mesh, pose);
}

void InteractiveSurfaceServer::add_selection_callback(SelectionCallback& f)
{
  selection_callbacks_.push_back(f);
}

void InteractiveSurfaceServer::clear_selection_callbacks() { selection_callbacks_.clear(); }

void InteractiveSurfaceServer::add_random_surface_marker()
{
  // create pose
  geometry_msgs::Pose pose;
  double xy_step = 0.2f;
  double z_step = 0.1f;
  double pmin = -2;
  double pmax = 2;
  tfScalar x = (pmax - pmin) * (static_cast<double>(rand()) / static_cast<double>(RAND_MAX)) + pmin;
  tfScalar y = (pmax - pmin) * (static_cast<double>(rand()) / static_cast<double>(RAND_MAX)) + pmin;
  ;
  tfScalar z = (pmax - pmin) * (static_cast<double>(rand()) / static_cast<double>(RAND_MAX)) + pmin;
  ;
  tf::Transform t = tf::Transform(tf::Quaternion::getIdentity(), tf::Vector3(x, y, z));
  tf::poseTFToMsg(t, pose);

  // create triangle list marker
  visualization_msgs::Marker marker;
  create_polygon_marker(marker, rand() % defaults::MAX_TRIANGLES + 1);

  add_surface(marker, pose);
}

void InteractiveSurfaceServer::create_polygon_marker(visualization_msgs::Marker& marker,
                                                     int triangles)
{

  ROS_INFO_STREAM("Creating polygon of " << triangles << " triangles");

  // setting marker properties
  marker.type = marker.TRIANGLE_LIST;
  marker.scale.x = marker.scale.y = marker.scale.z = 1;
  marker.pose.position.x = marker.pose.position.y = marker.pose.position.z = 0;
  marker.pose.orientation.x = marker.pose.orientation.y = marker.pose.orientation.z = 0;
  marker.pose.orientation.w = 1;
  marker.color.r = 0.5f;
  marker.color.g = 0.5f;
  marker.color.b = 0.8f;
  marker.color.a = 0.4f;

  // defining triangle vertices
  tf::Vector3 p0(-0.05f, 0, 0);
  tf::Vector3 p1(0.05f, 0, 0);
  tf::Vector3 p2;

  // rotation transform
  tf::Transform rot(tf::Quaternion(tf::Vector3(0, 0, 1), M_PI / 3.0f), tf::Vector3(0, 0, 0));

  for (int i = 0; i < triangles; i++)
  {
    // computing last triangle point
    p2 = rot * (p1 - p0) + p0;

    // creating temp vector
    std::vector<tf::Vector3> points;
    points.push_back(p0);
    points.push_back(p1);
    points.push_back(p2);

    // adding points to marker
    for (unsigned int j = 0; j < points.size(); j++)
    {
      geometry_msgs::Point p;
      p.x = points[j].getX();
      p.y = points[j].getY();
      p.z = points[j].getZ();
      marker.points.push_back(p);
    }

    // assigning new points for next triangle
    if (rand() % 2 == 0)
    {
      p1 = p2;
    }
    else
    {
      p0 = p2;
    }
  }
}

bool InteractiveSurfaceServer::rename_surface(const std::string& old_name,
                                              const std::string& new_name)
{
  std::map<std::string, bool>::iterator select_iter = surface_selection_map_.find(old_name);
  std::map<std::string, pcl::PolygonMesh>::iterator mesh_iter = meshes_map_.find(old_name);
  if (select_iter != surface_selection_map_.end() && mesh_iter != meshes_map_.end())
  {
    // The surface exists
    bool cache_is_selected = select_iter->second;
    pcl::PolygonMesh cache_mesh = mesh_iter->second;
    // Insert new item
    surface_selection_map_[new_name] = cache_is_selected;
    meshes_map_[new_name] = cache_mesh;
    // Remove old item
    meshes_map_.erase(mesh_iter);
    surface_selection_map_.erase(select_iter);
  }
  else
  {
    return false;
  }
}

} /* namespace interactive */
} /* namespace godel_surface_detection */
