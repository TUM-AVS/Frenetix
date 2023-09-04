#ifndef GEOMETRY_UTIL_H_
#define GEOMETRY_UTIL_H_

#pragma once
#include <Eigen/Dense>
#include <cmath>
#include <iostream>
#include <limits>
#include <vector>

namespace geometry {

typedef std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>>
    EigenPolyline;
using RowMatrixXd =
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;

namespace util {

int resample_polyline(const RowMatrixXd& polyline, double step,
                      RowMatrixXd& ret);

int resample_polyline(const geometry::EigenPolyline& polyline, double step,
                      geometry::EigenPolyline& ret);

int chaikins_corner_cutting(const RowMatrixXd& polyline, int refinements,
                            RowMatrixXd& ret);

int chaikins_corner_cutting(const geometry::EigenPolyline& polyline,
                            int refinements, geometry::EigenPolyline& ret);

enum class Orientation { COLINEAR, CLOCKWISE, COUNTERCLOCKWISE };

/**
 * Intersection between two lines defined by four points, algorithm from:
 * http://www.ahristov.com/tutorial/geometry-games/intersection-lines.html
 *
 * @param l1_pt1 first point of first line
 * @param l1_pt2 second point of first line
 * @param l2_pt1 first point of second line
 * @param l2_pt2 second point of second line
 * @param[out] intersection_point computed intersection point if lines intersect
 * @return True, if lines intersect, otherwise False
 */
static bool intersectionLineLine(const Eigen::Vector2d l1_pt1,
                                 const Eigen::Vector2d l1_pt2,
                                 const Eigen::Vector2d l2_pt1,
                                 const Eigen::Vector2d l2_pt2,
                                 Eigen::Vector2d& intersection_point) {
  double x1 = l1_pt1(0);
  double y1 = l1_pt1(1);
  double x2 = l1_pt2(0);
  double y2 = l1_pt2(1);
  double x3 = l2_pt1(0);
  double y3 = l2_pt1(1);
  double x4 = l2_pt2(0);
  double y4 = l2_pt2(1);

  double d = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);

  if (std::abs(d) <= 10e-8) {
    return false;
  }

  double xi =
      ((x3 - x4) * (x1 * y2 - y1 * x2) - (x1 - x2) * (x3 * y4 - y3 * x4)) / d;
  double yi =
      ((y3 - y4) * (x1 * y2 - y1 * x2) - (y1 - y2) * (x3 * y4 - y3 * x4)) / d;
  intersection_point = Eigen::Vector2d(xi, yi);
  return true;
}

/**
 * Given three colinear points p, q, r, the function checks if point q lies on
 * the line segment 'pr', algorithm from:
 * https://www.geeksforgeeks.org/check-if-two-given-line-segments-intersect/
 *
 * @param p start point of the segment
 * @param q point to be tested if it is on the segment
 * @param r end point of the segment
 * @return True, if point q lies on line segment 'pr', otherwise false
 */
static bool onSegmentGivenColinearPoints(Eigen::Vector2d p, Eigen::Vector2d q,
                                         Eigen::Vector2d r) {
  if (std::isgreaterequal(std::max(p[0], r[0]), q[0]) &&
      std::isgreaterequal(q[0], std::min(p[0], r[0])) &&
      std::isgreaterequal(std::max(p[1], r[1]), q[1]) &&
      std::isgreaterequal(q[1], std::min(p[1], r[1]))) {
    return true;
  } else {
    return false;
  }
}

/**
 * Determines if three points are ordered clockwise, counterclockwise, or
 * colinear. Reference:
 * https://www.geeksforgeeks.org/check-if-two-given-line-segments-intersect/
 *
 * @param p first point
 * @param q second point
 * @param r third point
 * @return orientation of points
 */
static Orientation determineOrientation(Eigen::Vector2d p, Eigen::Vector2d q,
                                        Eigen::Vector2d r) {
  double val = (q[1] - p[1]) * (r[0] - q[0]) - (q[0] - p[0]) * (r[1] - q[1]);

  if (val > 0.0) {
    return Orientation::CLOCKWISE;
  } else if (val < 0.0) {
    return Orientation::COUNTERCLOCKWISE;
  } else {
    return Orientation::COLINEAR;
  }
}

/**
 * Checks if two segments intersect, algorithm from:
 * https://www.geeksforgeeks.org/check-if-two-given-line-segments-intersect/
 *
 * @param p1 start point of first segment
 * @param q1 end point of first segment
 * @param p2 start point of second segment
 * @param q2 end point of second segment
 * @return True, if segments intersect, otherwise false
 */
static bool segmentsIntersect(Eigen::Vector2d p1, Eigen::Vector2d q1,
                              Eigen::Vector2d p2, Eigen::Vector2d q2) {
  Orientation o1 = determineOrientation(p1, q1, p2);
  Orientation o2 = determineOrientation(p1, q1, q2);
  Orientation o3 = determineOrientation(p2, q2, p1);
  Orientation o4 = determineOrientation(p2, q2, q1);

  if ((o1 != o2) && (o3 != o4)) {
    return true;
  }
  // Special Cases
  // p1, q1 and p2 are colinear and p2 lies on segment p1q1
  if (o1 == Orientation::COLINEAR and
      onSegmentGivenColinearPoints(p1, p2, q1)) {
    return true;
  }
  // p1, q1 and q2 are colinear and q2 lies on segment p1q1
  if (o2 == Orientation::COLINEAR and
      onSegmentGivenColinearPoints(p1, q2, q1)) {
    return true;
  }
  // p2, q2 and p1 are colinear and p1 lies on segment p2q2
  if (o3 == Orientation::COLINEAR and
      onSegmentGivenColinearPoints(p2, p1, q2)) {
    return true;
  }
  // p2, q2 and q1 are colinear and q1 lies on segment p2q2
  if (o4 == Orientation::COLINEAR and
      onSegmentGivenColinearPoints(p2, q1, q2)) {
    return true;
  }
  // Doesn't fall in any of the above cases
  return false;
}

/**
 * Computes the intersection point of two segments
 *
 * @param l1_pt1 start point of first segment
 * @param l1_pt2 end point of first segment
 * @param l2_pt1 start point of second segment
 * @param l2_pt2 end point of second segment
 * @param[out] intersection_point intersection point if segments intersect
 * @return True, if segments intersect, otherwise false
 */
static bool intersectionSegmentSegment(Eigen::Vector2d l1_pt1,
                                       Eigen::Vector2d l1_pt2,
                                       Eigen::Vector2d l2_pt1,
                                       Eigen::Vector2d l2_pt2,
                                       Eigen::Vector2d& intersection_point) {
  if (segmentsIntersect(l1_pt1, l1_pt2, l2_pt1, l2_pt2)) {
    return intersectionLineLine(l1_pt1, l1_pt2, l2_pt1, l2_pt2,
                                intersection_point);
  } else {
    return false;
  }
}

/**
 * Given three points p, q, r, the function checks if point q lies on the line
 * segment 'pr'.
 *
 * @param p start point of the segment
 * @param q point to be tested if it is on the segment
 * @param r end point of the segment
 * @return True, if point lies on segment; otherwise false
 */
static bool onSegment(Eigen::Vector2d p, Eigen::Vector2d q, Eigen::Vector2d r) {
  Orientation orientation = determineOrientation(p, q, r);
  if (orientation != Orientation::COLINEAR) {
    return false;
  }
  return onSegmentGivenColinearPoints(p, q, r);
}

/**
 * Projects a point onto new axes
 *
 * @param matr projection axes
 * @param point 2D point
 * @return projected point
 */
static Eigen::Vector2d projectOntoAxes(const Eigen::Matrix2d& matr,
                                       const Eigen::Vector2d& point) {
  return matr * point;
}

int to_RowMatrixXd(const geometry::EigenPolyline& polyline, RowMatrixXd& ret);
int to_EigenPolyline(const RowMatrixXd& polyline, geometry::EigenPolyline& ret);

}  // namespace util

}  // namespace geometry

#endif
