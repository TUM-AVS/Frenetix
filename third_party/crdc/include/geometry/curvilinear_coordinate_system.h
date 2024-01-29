#ifndef CURVILINEAR_COORDINATE_SYSTEM_H
#define CURVILINEAR_COORDINATE_SYSTEM_H

#include <algorithm>
#include <atomic>
#include <cmath>
#include <deque>
#include <limits>
#include <list>
#include <memory>
#include <numeric>
#include <vector>
#include <list>
#include <optional>

#include <Eigen/Dense>
#include <Eigen/StdVector>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/index/rtree.hpp>
#include <boost/geometry/geometries/register/box.hpp>

#include "geometry/application_settings.h"

#include "geometry/serialize/icurvilinear_coordinate_system_export.h"

#include "geometry/segment.h"
#include "geometry/util.h"


/*
 * Note:
 * The point_type_alias definition below is required because Boost.Geometry
 * already defines a point_type, but we also want to define a point_type.
 * Our point_type is going to shadow Boost's, however that doesn't work with
 * the BOOST_GEOMETRY_REGISTER_BOX macro, so we need to jump through a few hoops
 * here. Same for box_type vs box.
 *
 * TODO Use non-conflicting names or introduce namespace for our geometry types
 */
using point_type_alias = boost::geometry::model::d2::point_xy<double>;

/**
 * Box type (used for bounding boxes)
 *
 * Note: We don't use Boost's box model because the include is quite heavy and
 * boxes are (in)directly used almost everywhere.
 */
struct box_type {
    point_type_alias ll; //**< lower left corner */
    point_type_alias ur; //**< upper right corner */

    /**
     * Getter for lower left corner
     */
    point_type_alias min_corner() const noexcept { return ll; }

    /**
     * Getter for upper right corner
     */
    point_type_alias max_corner() const noexcept { return ur; }
};

// Register the box type for Boost

BOOST_GEOMETRY_REGISTER_BOX(box_type, point_type_alias, ll, ur)

namespace geometry {

class CurvilinearCoordinateSystem;
typedef std::shared_ptr<CurvilinearCoordinateSystem>
    CurvilinearCoordinateSystemPtr;
typedef std::shared_ptr<const CurvilinearCoordinateSystem>
    CurvilinearCoordinateSystemConstPtr;
}  // namespace geometry

namespace geometry {


using point_type = point_type_alias;

typedef boost::geometry::model::polygon<point_type> polygon_type;
typedef boost::geometry::model::multi_polygon<polygon_type> mpolygon_type;

/**
 * Quadrilateral used internally for representing the projection domain.
 */
struct quad {
    Eigen::Vector2d p1;
    Eigen::Vector2d p2;
    Eigen::Vector2d p3;
    Eigen::Vector2d p4;

    polygon_type polygon() const {
      polygon_type temp_poly;
      for (const auto &p : {p1, p2, p3, p4, p1}) {
        boost::geometry::append(temp_poly, point_type(p.x(), p.y()));
      }
      return temp_poly;
    }

    box_type box() const {
      polygon_type temp_poly = polygon();
      return boost::geometry::return_envelope<box_type>(temp_poly);
    }
};

enum class ProjectionAxis {
  X_AXIS = 0,
  Y_AXIS = 1,
  X_AXIS_ROTATED = 2,
  Y_AXIS_ROTATED = 3
};

/**
 * Projection domain errors are thrown when trying to convert coordinates
 * outside the projection domain.
 */
class ProjectionDomainError : public std::invalid_argument {
protected:
    explicit ProjectionDomainError(const std::string& message)
        : std::invalid_argument(message) {}
};

/**
 * Error for curvilinear coordinates outside the projection domain, either laterally or longitudinally.
 */
class CurvilinearProjectionDomainError : public ProjectionDomainError {
protected:
    explicit CurvilinearProjectionDomainError(const std::string& message)
        : ProjectionDomainError(message) {}

public:
   static CurvilinearProjectionDomainError longitudinal() {
       return CurvilinearProjectionDomainError("Longitudinal coordinate outside of projection domain");
   }

   static CurvilinearProjectionDomainError general() {
       return CurvilinearProjectionDomainError("Longitudinal and/or lateral coordinate outside of projection domain");
   }
};

/**
 * Error for cartesian coordinates outside the projection domain.
 */
class CartesianProjectionDomainError : public ProjectionDomainError {
protected:
    explicit CartesianProjectionDomainError(const std::string& message)
        : ProjectionDomainError(message) {}

public:
   static CartesianProjectionDomainError general() {
       return CartesianProjectionDomainError("x and/or y coordinate outside of projection domain");
   }
};

class CurvilinearCoordinateSystem
    : public std::enable_shared_from_this<CurvilinearCoordinateSystem> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * Creates a curvilinear coordinate system aligned to the given reference
   * path. The unique projection domain along the reference path is
   * automatically computed. The absolute value of the lateral distance of the
   * projection domain border from the reference path is limited to
   * default_projection_domain_limit. To account for numeric imprecisions, the
   * parameter eps reduces the computed lateral distance of the projection
   * domain border from the reference path.
   *
   * @param reference_path 2D polyline in Cartesian coordinates
   * @param default_projection_domain_limit maximum absolute distance of the
   * projection domain border from the reference path
   * @param eps reduces the lateral distance of the projection domain border
   * from the reference path
   * @param eps2 add three additional segments to the beginning the reference
   * path and two segments to the end, with length eps2, to enable the
   * conversion of points near the beginning and the end of the reference path
   */
  CurvilinearCoordinateSystem(const EigenPolyline& reference_path,
                              double default_projection_domain_limit = 20.,
                              double eps = 0.1, double eps2 = 1e-4);

  /**
   * Returns the reference path as a 2D polyline in Cartesian coordiantes
   * that has been extended according to the epsilon value. Please refer to the
   * class constructor parameter eps2 for more details.
   *
   * @return reference path to which the curvilinear coordinate system is
   * aligned to
   */
  EigenPolyline referencePath() const;

  /**
   * Returns the original unextended reference path as a 2D polyline in
   * Cartesian coordiantes. Please refer to the class constructor parameter eps2
   * for more details.
   *
   * @return reference path to which the curvilinear coordinate system is
   * aligned to
   */
  EigenPolyline referencePathOriginal() const;

  /**
   * Returns the border of the unique projection domain of the curvilinear
   * coordinate system in Cartesian coordinates
   *
   * @return 2D polyline representing the border of the projection domain
   */
  EigenPolyline projectionDomainBorder() const;

  /**
   * Returns the border of the unique projection domain of the curvilinear
   * coordinate system in curvilinear coordinates
   *
   * @return 2D line string representing the border of the projection domain
   */
  EigenPolyline curvilinearProjectionDomainBorder() const;

  /**
   * Returns the length of the reference path of the curvilinear coordinate
   * system
   *
   * @return length
   */
  double length() const;

  /**
   * Returns the maximum curvature radius along the reference path of the
   * curvilinear coordinate system
   *
   * @return maximum curvature radius
   */
  double maximumCurvatureRadius() const;

  /**
   * Returns the minimum curvature radius along the reference path of the
   * curvilinear coordinate system
   *
   * @return minimum curvature radius
   */
  double minimumCurvatureRadius() const;

  /**
   * Returns the maximum curvature along the reference path of the curvilinear
   * coordinate system
   *
   * @return maximum curvature
   */
  double maximumCurvature() const;

  /**
   * Returns the minimum curvature along the reference path of the curvilinear
   * coordinate system
   *
   * @return minimum curvature
   */
  double minimumCurvature() const;

  /**
   * Currently, the curvature of the reference path is not computed
   * automatically and must be set. Note that the validity of the curvature is
   * not checked, e.g., if it indeed corresponds to the reference path of the
   * curvilinear coordinate system.
   *
   * @param curvature of the reference path
   */
  void setCurvature(const std::vector<double> curvature);

  /**
   * Returns an interval of the curvatures of the reference path within a given
   * range of longitudinal positions.
   *
   * @param s_min minimum longitudinal position
   * @param s_max maximum longitudinal position
   * @return enclosing interval of curvature values of the reference path within
   * the range [s_min, s_max]
   */
  std::tuple<double, double> curvatureRange(double s_min, double s_max) const;
  std::vector<double> segmentsLongitudinalCoordinates() const;

  /**
   *
   * Normal vector at a specific longitudinal coordinate
   *
   * @param s longitudinal coordinate
   * @return normal vector
   */
  Eigen::Vector2d normal(double s) const;

  /**
   *
   * Computes the curvature for a given polyline
   *
   * @param polyline input polyline
   * @return curvature vector
   */

  static Eigen::VectorXd computeCurvature(const EigenPolyline &polyline);

  /**
   *
   * Computes and sets the curvature information for the reference path
   * @param digits: no. of decimal points for curvature value (default 8)
   *
   */

  int computeAndSetCurvature(int digits = 8);

  /**
   *
   * Tangent vector at a specific longitudinal coordinate
   *
   * @param s longitudinal coordinate
   * @return tangent vector
   */
  Eigen::Vector2d tangent(double s) const;

  /**
   * Transforms a point in the curvilinear coordinate frame to the global
   * coordinate frame.
   *
   * @param s longitudinal coordinate
   * @param l lateral coordinate
   * @return point in global coordinates
   */
  Eigen::Vector2d convertToCartesianCoords(double s, double l) const;

  /**
   * Transforms a Cartesian point to the curvilinear frame.
   *
   * @param x x-coordinate in the Cartesian coordinate system
   * @param y y-coordinate in the Cartesian coordinate system
   * @return point in the curvilinear frame.
   */
  Eigen::Vector2d convertToCurvilinearCoords(double x, double y) const;

  /**
   * Transforms a Cartesian point to the curvilinear frame and returns the
   * segment index, in which the point is contained.
   *
   * @param[in] x x-coordinate in the Cartesian coordinate system
   * @param[in] y y-coordinate in the Cartesian coordinate system
   * @param[out] segment index, in which the point is contained
   * @return point in the curvilinear frame.
   */
  Eigen::Vector2d convertToCurvilinearCoordsAndGetSegmentIdx(
      double x, double y, int &segment_idx) const;

  /**
   * Transforms a rectangle in the curvilinear coordinates to the Cartesian
   * coordinates. Additionally, a triangle mesh of the resulting polygon is
   * generated.
   *
   * @param[in] s_lo minimum longitudinal coordinate of the rectangle
   * @param[in] s_hi maximum longitudinal coordinate of the rectangle
   * @param[in] l_lo minimum lateral coordinate of the rectangle.
   * @param[in] l_hi maximum lateral coordinate of the rectangle
   * @param[out] triangle_mesh
   * @return transformed rectangle in Cartesian coordinates
   */
  EigenPolyline convertRectangleToCartesianCoords(
      double s_lo, double s_hi, double l_lo, double l_hi,
      std::vector<EigenPolyline> &triangle_mesh) const;

  /**
   * Converts list of points to the curvilinear coordinate system.
   *
   * @param points vector of points in the global coordinate frame
   * @param num_omp_threads number of OMP threads for computation
   * @return transformed points
   */
  EigenPolyline convertListOfPointsToCurvilinearCoords(
      const EigenPolyline &points, int num_omp_threads) const;

  /**
   * Converts list of points to the cartesian coordinate system
   *
   * @param points vector of points in the curvilinear coordinate system
   * @param num_omp_threads number of OMP threads for computation
   * @return transformed points
   */
  EigenPolyline convertListOfPointsToCartesianCoords(
          const EigenPolyline &points, int num_omp_threads) const;


  /**
   * Transforms a polygon to the curvilinear coordinate system.
   *
   * @param[in] polygon
   * @param[out] transformed_polygon transformed polygon
   */
  void convertPolygonToCurvilinearCoords(
      const EigenPolyline &polygon,
      std::vector<EigenPolyline> &transformed_polygon) const;

  /**
   * Transforms different multipolygons which are inside the unique projection
   * domain of the curvilinear coordinate system. Furthermore, the transformed
   * multipolygons are rasterized meaning that the polygons are
   * over-approximated with axis-aligned rectangles.
   *
   * @param[in] polygons list of polygons
   * @param[in] groups_of_polygons list of IDs indicating the group to which a
   * polygon belongs
   * @param[in] num_polygon_groups number of different polygon groups
   * @param[in] num_omp_threads number of OMP threads for computation
   * @param[out] transformed_polygons transformed polygons
   * @param[out] transformed_polygons_rasterized transformed and rasterized
   * polygons
   */
  void convertListOfPolygonsToCurvilinearCoordsAndRasterize(
      const std::vector<EigenPolyline> &polygons,
      const std::vector<int> groups_of_polygons, int num_polygon_groups,
      int num_omp_threads,
      std::vector<std::vector<EigenPolyline>> &transformed_polygons,
      std::vector<std::vector<EigenPolyline>> &transformed_polygons_rasterized)
      const;

  /**
   * Validates if a point in global coordinates is within the unique projection
   domain of the curvilinear coordinate
   * system.
   *
   * @param x x-coordinate in the Cartesian coordinate system
   * @param y y-coordinate in the Cartesian coordinate system
   * @return true, if the point is inside or on the boundary of the projection
   domain, false, if the point is outside of the boundary of the projection
   domain.
   */
  bool cartesianPointInProjectionDomain(double x, double y) const;

  /**
   * Validates if a point in global coordinates is within the unique projection
   domain of the curvilinear coordinate
   * system.
   *
   * @param x x-coordinate in the Cartesian coordinate system
   * @param y y-coordinate in the Cartesian coordinate system
   * @return true, if the point is inside or on the boundary of the projection
   domain, false, if the point is outside of the boundary of the projection
   domain.
    */
  bool curvilinearPointInProjectionDomain(double s, double l) const;

  /**
   * Computes the parts of a polygon which are inside the unique projection
   domain of the curvilinear coordinate system.
   *
   * @param polygon vertices of the boundary of the polygon; vertices must be
   sorted clockwise and given as closed list (the last vertex must be the same
   as the first one)
   * @return parts of the polygon which are inside the projection domain.
   */
  std::vector<EigenPolyline> determineSubsetOfPolygonWithinProjectionDomain(
      const EigenPolyline &polygon) const;

  /**
   * Computes the parts of a polygon (given in curvilinear coordinates) which
   are inside the curvilinear projection
   * domain of the curvilinear coordinate system.
   *
   * @param polygon vertices of the boundary of the polygon; vertices must be
   sorted clockwise and given as closed list (the last vertex must be the same
   as the first one)
   * @return parts of the polygon which are inside the curvilinear projection
   domain.
   */
  std::vector<EigenPolyline>
  determineSubsetOfPolygonWithinCurvilinearProjectionDomain(
      const EigenPolyline &polygon) const;


  /**
   * Computes the parts of different multipolygons which are inside the unique
   * projection domain of the curvilinear coordinate system.
   *
   * @param[in] polygons list of polygons
   * @param[in] groups_of_polygons list of IDs indicating the group to which a
   * polygon belongs
   * @param[in] num_omp_threads number of OMP threads for computation
   * @param[out] polygons_in_projection_domain polygons within the projection
   * domain
   * @param[out] groups_of_polygons_in_projection_domain indices indicating to
   * which group the clipped polygon belongs
   */
  void determineSubsetsOfMultiPolygonsWithinProjectionDomain(
      const std::vector<EigenPolyline> &polygons,
      const std::vector<int> groups_of_polygons, const int num_omp_threads,
      std::vector<EigenPolyline> &polygons_in_projection_domain,
      std::vector<int> &groups_of_polygons_in_projection_domain) const;

  double defaultProjectionDomainLimit() const {
    return default_projection_domain_limit_;
  }

  double eps() const { return eps_; }

  double eps2() const { return eps2_; }

  std::vector<double> curvatureVector(void) const { return curvature_vec_; }

#if ENABLE_SERIALIZER
  serialize::ICurvilinearCoordinateSystemExport *exportThis(void) const;

  int serialize(std::ostream &output_stream) const;
  static CurvilinearCoordinateSystemConstPtr deserialize(
      std::istream &output_stream);

#endif

 private:
  static Eigen::VectorXd gradient(const Eigen::VectorXd &input);

  /**
   * Converts groups of points to the curvilinear coordinate system.
   *
   * @param groups_of_points vector of point clouds in the global coordinate
   * frame
   * @param num_omp_threads number of OMP threads for computation
   * @return groups of transformed points and corresponding segment index, in
   * which the points are contained
   */
  std::vector<std::vector<std::tuple<int, double, double>>>
  convertToCurvilinearCoords(const std::vector<EigenPolyline> &groups_of_points,
                             int num_omp_threads) const;

  /**
   * Computes the parts of a polygon which are inside the unique projection
   * domain of the curvilinear coordinate system. The projection domain can be
   * either the curvilinear or global projection domain.
   *
   * @param polygon vertices of the boundary of the polygon; vertices must be
   * sorted clockwise and given as closed list (the last vertex must be the same
   * as the first one)
   * @param projection_domain curvilinear or global projection domain of
   * coordinate system
   * @return parts of the polygon which are inside the projection domain.
   */
  std::vector<EigenPolyline> polygonWithinProjectionDomain(
      const EigenPolyline &polygon,
      const polygon_type &projection_domain) const;

  /**
   * Creates a new segment of the curvilinear coordinate system.
   *
   * @param p_1 start point of the segment
   * @param p_2 end point of the segment
   * @param t_1 tangent vector at the start of the segment
   * @param t_2 tangent vector at the end of the segment
   */
  void createSegment(const Eigen::Vector2d& pt_1, const Eigen::Vector2d& pt_2,
                     const Eigen::Vector2d& t_1,  const Eigen::Vector2d& t_2);

  /**
   * Approximates the unique projection domain of the coordinate system.
   *
   * @param eps parameter to shrink the projection domain which avoids numerical
   * errors
   */
  void approximateProjectionDomain(double eps = 0.1);

  /**
   * Approximates the curvilinear projection domain of the coordinate system.
   */
  void approximateCurvilinearProjectionDomain();

  /**
   * Computes the maximum positive and negative distance from the reference
   * path, for which points can be transformed uniquely.
   *
   * @param eps parameter to shrink the projection domain which avoids numerical
   * errors
   * @return minimum and maximum distance
   */
  std::tuple<double, double> computeProjectionDomainLimits(double eps) const;

  /**
   * Computes the border of the unique projection domain. The points are sorted
   * clockwise and the first point coincides with the last point.
   *
   * @param min_radius maximum distance of border to the right of the reference
   * path
   * @param max_radius minimum distance of border to the left of the reference
   * path
   * @return border points of projection domain.
   */
  EigenPolyline computeProjectionDomainBorder(double min_radius,
                                              double max_radius);

  /**
   * Computes a projection axis for each segment to quickly determine global
   * points that lie within the segment. Only the points that lie between the
   * minimum and the maximum coordinates projected on the axis are considered to
   * be within the segment.
   *
   * First, the intersections between the normals n_1_ and n_2_ of the segment
   * and the projection domain border are found, resulting in 4 intersection
   * points. Second, the starting point pt_1_ , the end point pt_2_, as well as
   * the found intersection points of the segment are projected onto the x- and
   * y-axes of the global coordinate system. All candidate points for the
   * projection onto the segment must lie within the bounding box (x_min; x_max;
   * y_min; y_max). In addition, the six points are projected onto the two
   * diagonal axes that are obtained through rotating the x and y axes
   * counterclockwise by 45 degrees. The best axis for each segment is the axis
   * for which the difference between the maximum and the minimum projection
   * coordinate is minimal.
   *
   */
  void computeBestProjectionAxisForSegments();

  /**
   * The function computes the corresponding curvilinear points of a polygon for
   * intermediate segments.
   *
   * @param polygon polygon boundary in global coordinates
   * @param curvilinear_coordinates_and_segment_idx transformed boundary of
   * polygons and corresponding segment indices
   * @param curvilinear_polygon complete transformed boundary of polygon
   * @param segment_indices
   * @return transformed polygon
   */
  bool addPointsAtSegmentTransition(
      const EigenPolyline &polygon,
      const std::vector<std::tuple<int, double, double>>
          &curvilinear_coordinates_and_segment_idx,
      EigenPolyline &curvilinear_polygon, std::set<int> &segment_indices) const;

  /**
   * Given two Cartesian points (cartesian_point and next_cartesian_point), the
   * function computes the intersection points of the line connecting the two
   * points and the segment normals.
   *
   * @param[in] cartesian_point
   * @param[in] next_cartesian_point
   * @param[in] indices_range IDs of segments whose normals intersect with the
   * line connecting cartesian_point and next_cartesian_point
   * @param[in/out] transformed_polygon transformed intersection points are
   * appended to transformed_polygon
   */
  void interpolatePointsBetweenSegments(
      const Eigen::Vector2d cartesian_point,
      const Eigen::Vector2d next_cartesian_point,
      const std::list<int> indices_range,
      EigenPolyline &transformed_polygon) const;

  /**
   * Given two segment indices, the function computes the intermediate indices.
   *
   * @param segment_idx
   * @param previous_segment_idx
   * @return segment indices from segment_idx to previous_segment_idx
   */
  std::list<int> determineIndicesRange(int segment_idx,
                                       int previous_segment_idx) const;

  /**
   * Over-approximates polygons in the curvilinear coordinate system with
   * axis-aligned rectangles.
   *
   * @param transformed_polygon polygons in curvilinear coordinates
   * @param segment_indices IDs of all segments containing all transformed
   * polygons
   * @param transformed_polygon_rasterized over-approximated polygons
   */
  void rasterizeListOfTransformedPolygonsInProjectionDomain(
      const std::vector<EigenPolyline> &transformed_polygon,
      const std::set<int> &segment_indices,
      std::vector<EigenPolyline> &transformed_polygon_rasterized) const;

  /**
   * Over-approximates a polygon in the curvilinear coordinate system with
   * axis-aligned rectangles.
   *
   * @param transformed_polygon polygon in curvilinear coordinates
   * @param segment_indices  IDs of all segments containing the transformed
   * polygon
   * @param transformed_polygon_rasterized over-approximated polygon
   */
  void rasterizeTransformedPolygonInProjectionDomain(
      const EigenPolyline &transformed_polygon,
      const std::set<int> &segment_indices,
      std::vector<EigenPolyline> &transformed_polygon_rasterized) const;

  /**
   * Given a list of coordinates projected to the corresponding projection axis
   * of a segment, the function returns the indices of the points which may be
   * inside the segment.
*
   * @param segment_idx ID of the segment
   * @param pair_projected_coord_and_id projected coordinates to the projection
   * axis of the segment with ID segment_idx
   *                                    + unique ID for each projected
   * coordinate
   * @return IDs of projected coordinates which are candidate that lie inside
   * the segment with ID segment_idx
   */
  std::vector<int> findCandidatePointsInSegment(
      int segment_idx,
      const std::vector<std::pair<double, int>> &pair_projected_coord_and_id)
      const;

  /**
   * Given a set of candidate segments for each point, the transformed
   * longitudinal and lateral coordinates for each candidate segment, the
   * function computes the curvilinear coordinates.
   */
  void determineCurvilinearCoordinatesAndSegmentIdx(
      const std::vector<std::vector<std::tuple<int, int>>>
          &candidate_segments_of_points,
      const std::vector<Eigen::RowVectorXd,
                        Eigen::aligned_allocator<Eigen::RowVectorXd>> &s_coord,
      const std::vector<Eigen::RowVectorXd,
                        Eigen::aligned_allocator<Eigen::RowVectorXd>> &l_coord,
      int num_omp_threads,
      std::vector<std::vector<std::tuple<int, double, double>>>
          &groups_of_curvil_points) const;

  /**
   * Finds the corresponding segment for a given longitudinal coordinate of a
   * point.
   *
   * @param s longitudinal coordinate
   * @return segment index
   */
  int findSegmentIndex(double s) const;

  /**
   * Finds the corresponding segment for a given longitudinal coordinate of a
   * point.
   *
   * @param s longitudinal coordinate
   * @return segment index, or std::nullopt if not found
   */
  std::optional<int> tryFindSegmentIndex(double s) const;


  void removeSegment(int ind);

 private:
  std::optional<int> findSegmentIndex_Fast(double s) const;
  std::optional<int> findSegmentIndex_Slow(double s) const;

  EigenPolyline reference_path_original_;
  EigenPolyline reference_path_;

  std::vector<std::unique_ptr<Segment>> segment_list_;
  std::vector<double> segment_longitudinal_coord_;
  polygon_type projection_domain_;
  polygon_type curvilinear_projection_domain_;

  // R*-tree is better here: slow insertion, but fast lookup
  using index_params = boost::geometry::index::rstar<8>;
  using quadtree_value_type = std::pair<box_type, quad>;
  using quadtree_type = boost::geometry::index::rtree<quadtree_value_type, index_params>;
  quadtree_type projection_domain_quads_;
  quadtree_type curvilinear_projection_domain_quads_;
  bool curvilinear_projection_domain_quads_valid_ = false;

  EigenPolyline upper_projection_domain_border_;
  EigenPolyline lower_projection_domain_border_;

  std::vector<double> min_best_segm_axis_;
  std::vector<double> max_best_segm_axis_;
  std::deque<ProjectionAxis> best_segm_axis_;

  double length_;
  double max_curvature_radius_;
  double min_curvature_radius_;
  double max_curvature_;
  double min_curvature_;
  /// curvature value at longitudinal positions of reference path
  std::map<double, double> curvature_;
  double default_projection_domain_limit_;  //25 m
  double eps_; // reduction
  double eps2_;

  std::vector<double> curvature_vec_;
};

}  // namespace geometry

#endif  // CURVILINEAR_COORDINATE_SYSTEM_H
