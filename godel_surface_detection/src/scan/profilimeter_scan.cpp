#include "godel_surface_detection/scan/profilimeter_scan.h"
#include <ros/ros.h>
using namespace godel_process_path;

// Local types
namespace
{
  // stub for gift-wrapping algorithm to compute convex hull
  // stub for rotating-calipers algorithm to compute min rectangle

  /**
   * Rotated rectangle structure for use with striping algorithm
   * (x,y) is the center (of mass)
   * w, h are the edge lengths
   * a is the clockwise rotation from upright
   */
  struct RotatedRect
  {
    double x, y, w, h, a;
  };

  RotatedRect simpleBoundingBox(const PolygonBoundary& boundary)
  {
    double min_x = boundary[0].x;
    double min_y = boundary[0].y;
    double max_x = boundary[0].x;
    double max_y = boundary[0].y;

    for (std::size_t i = 1; i < boundary.size(); ++i)
    {
      const PolygonPt& pt = boundary[i];
      min_x = std::min(pt.x, min_x);
      min_y = std::min(pt.y, min_y);
      max_x = std::max(pt.x, max_x);
      max_y = std::max(pt.y, max_y); 
    }

    RotatedRect result;
    result.a = 0.0; // non-rotated, hence the simple bounding box
    result.x = (max_x + min_x) / 2;
    result.y = (max_y + min_y) / 2;
    result.w = max_x - min_x;
    result.h = max_y - min_y;
    return result;
  }

  // Vertically or horizontally?
  std::vector<RotatedRect> sliceBoundingBox(const RotatedRect& bbox, double slice_width, double overlap)
  {
    // we assume slice_width > overlap
    int n_slices;
    float dx, dy; // unit vector
    float x, y; // start position
    float w, h;

    float adjusted_width = slice_width - overlap;

    // Determine which way the slice should go
    if (bbox.w < bbox.h)
    {
      // walk horizontally, vertical slices
      n_slices = static_cast<int>(bbox.w / (adjusted_width)) + 1;
      
      dx = std::cos(bbox.a);
      dy = std::sin(bbox.a);

      x = bbox.x - dx * (bbox.w - adjusted_width) / 2.0;
      y = bbox.y - dy * (bbox.w - adjusted_width) / 2.0;

      w = adjusted_width;
      h = bbox.h;
    }
    else
    {
      // walk vertically, horizontal slices
      n_slices = static_cast<int>(bbox.h / (adjusted_width)) + 1;
      
      dx = -std::sin(bbox.a);
      dy = std::cos(bbox.a);

      x = bbox.x - dx * (bbox.h - adjusted_width) / 2.0;
      y = bbox.y - dy * (bbox.h - adjusted_width) / 2.0; 

      w = bbox.w;
      h = adjusted_width;
    }
    
    std::vector<RotatedRect> slices;
    slices.reserve(n_slices);

    dx *= adjusted_width;
    dy *= adjusted_width;

    for (int i = 0; i < n_slices; ++i)
    {
      RotatedRect temp;
      temp.a = bbox.a;
      temp.x = x;
      temp.y = y;
      temp.w = w;
      temp.h = h;
      slices.push_back(temp);
      x += dx;
      y += dy;
    }

    return slices;
  }

  std::vector<PolygonPt> interpolateAlongAxis(const RotatedRect& rect, double ds)
  {
    std::vector<PolygonPt> pts;
    double dx, dy; // 
    float x, y;
    int n;
    if (rect.w > rect.h)
    {
      n = rect.w / ds;
      dx = std::cos(rect.a);
      dy = std::sin(rect.a);
      x = rect.x - dx * rect.w/2.0;
      y = rect.y - dy * rect.w/2.0; 
    }
    else
    {
      n = rect.h / ds;
      dx = -std::sin(rect.a);
      dy = std::cos(rect.a);
      x = rect.x - dx * rect.h/2.0;
      y = rect.y - dy * rect.h/2.0;
    }

    // pts.reserve(n);

    // transform unit vector
    dx *= ds; 
    dy *= ds;

    for (int i = 0; i < n; ++i)
    {
      PolygonPt pt;
      pt.x = x;
      pt.y = y;
      x += dx;
      y += dy;
      pts.push_back(pt);
    }

    return pts;
  }

  std::vector<PolygonPt> makeStitch(const PolygonPt& a, const PolygonPt& b, double ds)
  {
    double dx = b.x - a.x;
    double dy = b.y - a.y;
    double s = std::sqrt(dx*dx + dy*dy);
    // steps
    size_t n = s / ds;
    // unit vector
    dx /= s;
    dy /= s;

    std::vector<PolygonPt> result;
    for (size_t i = 0; i < n; ++i)
    {
      PolygonPt pt;
      pt.x = a.x + i * dx * ds;
      pt.y = a.y + i * dy * ds;
      result.push_back(pt);
    }
    return result;
  }

  std::vector<PolygonPt> stitchAndFlatten(const std::vector<std::vector<PolygonPt> >& paths, double ds)
  {
    std::vector<PolygonPt> result;
    bool forward = true;
    for (std::size_t i = 0; i < paths.size(); ++i)
    {
      // copy current path in given direction
      if (forward)
      {
        result.insert(result.end(), paths[i].begin(), paths[i].end());
      }
      else
      {
        result.insert(result.end(), paths[i].rbegin(), paths[i].rend());
      }

      forward = !forward;

      // check to see if there is a next strip
      if (i+1 < paths.size())
      {
        // if so, stitch to the next point
        std::vector<PolygonPt> stitch = makeStitch(*result.rbegin(), (forward ? *(paths[i+1].begin()) : *(paths[i+1].rbegin())), ds);
        result.insert(result.end(), stitch.begin(), stitch.end());
      }
    }

    return result;
  }


} // end anon namespace

void printPt(const PolygonPt& pt)
{
  ROS_WARN("%f %f", pt.x, pt.y);
}

std::vector<PolygonPt> 
godel_surface_detection::generateProfilimeterScanPath(const PolygonBoundary& boundary,
                                                      const godel_msgs::ScanPlanParameters &params)
{
  static const double DS = 0.01;

  // Step 1 -> compute (oriented?) bounding box
  RotatedRect bbox = simpleBoundingBox(boundary);
  
  // Step 2 -> slice bounding box into strips
  std::vector<RotatedRect> slices = sliceBoundingBox(bbox, params.scan_width, params.overlap);
  
  // Step 3 -> generate set of dense points along center of each strip
  std::vector<std::vector<PolygonPt> > slice_points;
  slice_points.reserve(slices.size());
  for (std::size_t i = 0; i < slices.size(); ++i)
    slice_points.push_back(interpolateAlongAxis(slices[i], DS));

  // Step 4 -> connect slices together
  std::vector<PolygonPt> pts = stitchAndFlatten(slice_points, DS); 

  return pts;
}
