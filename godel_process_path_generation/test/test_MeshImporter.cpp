/*
 * test_MeshImporter.cpp
 *
 *  Created on: May 8, 2014
 *      Author: Dan Solomon
 */

#include <gtest/gtest.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include "godel_process_path_generation/mesh_importer.h"
#include "test_common_functions.h"

/* Test fixture */

typedef pcl::PointCloud<pcl::PointXYZINormal> CloudN;
using godel_process_path::Cloud;
using godel_process_path::PolygonPt;
using godel_process_path::PolygonBoundary;
using godel_process_path::PolygonBoundaryCollection;


class MeshImporterTest: public ::testing::Test
{
protected:

  // x,y,z = 0,1,2
  void createMesh(int dir)
  {
    pointcloud_ = CloudN::Ptr(new CloudN);
    mesh_ = pcl::PolygonMeshPtr(new pcl::PolygonMesh);
    expected_boundary_.pts.clear();

    pcl::PointXYZINormal p0, p1, p2, p3, p4;
    /*  0---1
     *  |\ /|
     *  | 2 |
     *  |/ \|
     *  3---4 */
    if (dir == 0)       //normal in X-direction
    {
             p0.x=0; p0.y=1;  p0.z=1;  p0.normal_z = p0.normal_y = 0; p0.normal_x = 1; pointcloud_->push_back(p0);
      p1=p0; p1.x=0; p1.y=1;  p1.z=-1; pointcloud_->push_back(p1);
      p2=p0; p2.x=0; p2.y=0;  p2.z=0;  pointcloud_->push_back(p2);
      p3=p0; p3.x=0; p3.y=-1; p3.z=1;  pointcloud_->push_back(p3);
      p4=p0; p4.x=0; p4.y=-1; p4.z=-1; pointcloud_->push_back(p4);

      expected_boundary_.pts.push_back(PolygonPt(-1, 1));
      expected_boundary_.pts.push_back(PolygonPt( 1, 1));
      expected_boundary_.pts.push_back(PolygonPt( 1,-1));
      expected_boundary_.pts.push_back(PolygonPt(-1,-1));
    }
    else if (dir == 1)  // normal in Y-direction
    {
             p0.x=-1; p0.y=0; p0.z=1;  p0.normal_x = p0.normal_z = 0; p0.normal_y = 1; pointcloud_->push_back(p0);
      p1=p0; p1.x=-1; p1.y=0; p1.z=-1; pointcloud_->push_back(p1);
      p2=p0; p2.x=0;  p2.y=0; p2.z=0;  pointcloud_->push_back(p2);
      p3=p0; p3.x=1;  p3.y=0; p3.z=1;  pointcloud_->push_back(p3);
      p4=p0; p4.x=1;  p4.y=0; p4.z=-1; pointcloud_->push_back(p4);

      expected_boundary_.pts.push_back(PolygonPt(-1, 1));
      expected_boundary_.pts.push_back(PolygonPt( 1, 1));
      expected_boundary_.pts.push_back(PolygonPt( 1,-1));
      expected_boundary_.pts.push_back(PolygonPt(-1,-1));
    }
    else                // normal in Z-direction
    {
             p0.x=-1; p0.y=1;  p0.z=0; p0.normal_x = p0.normal_y = 0; p0.normal_z = 1; pointcloud_->push_back(p0);
      p1=p0; p1.x=1;  p1.y=1;  p1.z=0; pointcloud_->push_back(p1);
      p2=p0; p2.x=0;  p2.y=0;  p2.z=0; pointcloud_->push_back(p2);
      p3=p0; p3.x=-1; p3.y=-1; p3.z=0; pointcloud_->push_back(p3);
      p4=p0; p4.x=1;  p4.y=-1; p4.z=0; pointcloud_->push_back(p4);

      expected_boundary_.pts.push_back(PolygonPt(-1,-1));
      expected_boundary_.pts.push_back(PolygonPt(-1, 1));
      expected_boundary_.pts.push_back(PolygonPt( 1, 1));
      expected_boundary_.pts.push_back(PolygonPt( 1,-1));
    }

    pcl::toPCLPointCloud2(*pointcloud_, mesh_->cloud);
    pcl::Vertices polygon;

    polygon.vertices.clear();
    polygon.vertices.push_back(0); polygon.vertices.push_back(2); polygon.vertices.push_back(1);
    mesh_->polygons.push_back(polygon);

    polygon.vertices.clear();
    polygon.vertices.push_back(1); polygon.vertices.push_back(2); polygon.vertices.push_back(4);
    mesh_->polygons.push_back(polygon);

    polygon.vertices.clear();
    polygon.vertices.push_back(4); polygon.vertices.push_back(2); polygon.vertices.push_back(3);
    mesh_->polygons.push_back(polygon);

    polygon.vertices.clear();
    polygon.vertices.push_back(3); polygon.vertices.push_back(2); polygon.vertices.push_back(0);
    mesh_->polygons.push_back(polygon);
  }

  virtual void SetUp()
  {
  }

  CloudN::Ptr pointcloud_;
  pcl::PolygonMeshPtr mesh_;
  PolygonBoundary expected_boundary_;
};

typedef MeshImporterTest calcBoundaryData;

TEST_F(calcBoundaryData, fixedMeshes)
{
  godel_process_path::MeshImporter mi;
  mi.verbose_ = true;

  // Flat mesh with Normal along Z
  createMesh(2);
  EXPECT_TRUE(mi.calculateBoundaryData(*mesh_));
  PolygonBoundaryCollection actual_collection = mi.getBoundaries();
  EXPECT_EQ(actual_collection.size(), 1);

  PolygonBoundary actual = actual_collection.front();
  EXPECT_TRUE(::isCircularPermutation(actual.pts, expected_boundary_.pts));
//  std::cout << "Actual:\n" << actual << std::endl << "Expected:\n" << expected_boundary_ << std::endl;


  // Flat mesh with Normal along X
  createMesh(0);
  EXPECT_TRUE(mi.calculateBoundaryData(*mesh_));
  actual_collection = mi.getBoundaries();
  EXPECT_EQ(actual_collection.size(), 1);

  actual = actual_collection.front();
  EXPECT_TRUE(::isCircularPermutation(actual.pts, expected_boundary_.pts));
//  std::cout << "Actual:\n" << actual << std::endl << "Expected:\n" << expected_boundary_ << std::endl;

  // Flat mesh with Normal along Y
  createMesh(1);
  EXPECT_TRUE(mi.calculateBoundaryData(*mesh_));
  actual_collection = mi.getBoundaries();
  EXPECT_EQ(actual_collection.size(), 1);

  actual = actual_collection.front();
  EXPECT_TRUE(::isCircularPermutation(actual.pts, expected_boundary_.pts));
//  std::cout << "Actual:\n" << actual << std::endl << "Expected:\n" << expected_boundary_ << std::endl;
}


int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
