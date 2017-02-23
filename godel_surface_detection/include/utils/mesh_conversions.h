#ifndef GODEL_MESH_CONVERSIONS_H
#define GODEL_MESH_CONVERSIONS_H

#include <geometry_msgs/Point.h>
#include <pcl_ros/point_cloud.h>

namespace godel_surface_detection
{

/**
 * @brief Flattens the list of vertices and polygon-indices found in the pcl::PolygonMesh data
 * structure into a simple list of 3d positions. Meant for serializing mesh data into a
 * visualization_msgs::Marker TRIANGLE_LIST.
 * @param mesh Source of our vertex and index data. We currently only handle 3 and 4 sided polygons.
 * @param points Output parameter where each sequential tuple of 3 points is a triangle.
 */
void meshToTrianglePoints(const pcl::PolygonMesh& mesh, std::vector<geometry_msgs::Point>& points);

/**
 * @brief Converts a flattened triangle list into a pcl::PolygonMesh object. DOES NOT attempt
 * to reduce the vertex count.
 * @param points A sequence of points where each sequential set of 3 points is a triangle.
 * @param mesh A polygon mesh of the triangles in \e points. If any triangles are connected,
 * this will produce duplicate points.
 */
void trianglePointsToMesh(const std::vector<geometry_msgs::Point>& points, pcl::PolygonMesh& mesh);

}

#endif // GODEL_MESH_CONVERSIONS_H
