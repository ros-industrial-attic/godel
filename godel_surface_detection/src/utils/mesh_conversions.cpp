#include "utils/mesh_conversions.h"

void godel_surface_detection::meshToTrianglePoints(const pcl::PolygonMesh &mesh, std::vector<geometry_msgs::Point> &points)
{
  pcl::PointCloud<pcl::PointXYZ> mesh_points;
  pcl::fromPCLPointCloud2(mesh.cloud, mesh_points);

  auto add_point = [&mesh_points](uint32_t idx)
  {
    geometry_msgs::Point p;
    p.x = mesh_points.points[idx].x;
    p.y = mesh_points.points[idx].y;
    p.z = mesh_points.points[idx].z;
    return p;
  };

  for (std::size_t i = 0; i < mesh.polygons.size(); ++i)
  {
    const pcl::Vertices& v = mesh.polygons[i];

    // We currently support 3 and 4-gon meshes
    if (v.vertices.size() == 3)
    {
      points.push_back(add_point(v.vertices[0]));
      points.push_back(add_point(v.vertices[1]));
      points.push_back(add_point(v.vertices[2]));
    }
    else if (v.vertices.size() == 4)
    {
      auto v1 = v.vertices[0];
      auto v2 = v.vertices[1];
      auto v3 = v.vertices[2];
      auto v4 = v.vertices[3];

      // Add triangle 1
      points.push_back(add_point(v1));
      points.push_back(add_point(v2));
      points.push_back(add_point(v3));

      // triangle 2
      points.push_back(add_point(v3));
      points.push_back(add_point(v4));
      points.push_back(add_point(v1));
    }
    else
    {
      const auto sz = std::to_string(v.vertices.size());
      throw std::invalid_argument("meshToTrianglePoints(): passed polygon w/ " + sz + " vertices. Only supports 3 and 4.");
    }
  } // end of vertex loop
}

void godel_surface_detection::trianglePointsToMesh(const std::vector<geometry_msgs::Point> &points, pcl::PolygonMesh &mesh)
{
  pcl::PointCloud<pcl::PointXYZ> mesh_points;

  for (std::size_t i = 0; i < points.size(); i += 3)
  {
    pcl::Vertices v;
    for (int j = 0; j < 3; j++)
    {
      v.vertices.push_back(i + j);

      pcl::PointXYZ p;
      p.x = points[i + j].x;
      p.y = points[i + j].y;
      p.z = points[i + j].z;
      mesh_points.points.push_back(p);
    }
    mesh.polygons.push_back(v);
  }

  pcl::toPCLPointCloud2(mesh_points, mesh.cloud);
}
