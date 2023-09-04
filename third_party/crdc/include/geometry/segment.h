#ifndef SEGMENT_H
#define SEGMENT_H

#include <algorithm>
#include <utility>
#include <vector>

#include <Eigen/Dense>

#include "geometry/util.h"

namespace geometry {

class CurvilinearCoordinateSystem;  // forward declaration

class Segment {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  friend class CurvilinearCoordinateSystem;

  /**
   * Constructor.
   *
   * @param p_1 start point of the segment
   * @param p_2 end point of the segment
   * @param t_1 tangent vector at the start of the segment
   * @param t_2 tangent vector at the end of the segment
   */
  Segment(const Eigen::Vector2d& pt_1, const Eigen::Vector2d& pt_2, const Eigen::Vector2d& t_1,
          const Eigen::Vector2d& t_2);

  /**
   * @return start point of segment in Cartesian coordinates
   */
  Eigen::Vector2d pt_1() const;

  /**
   * @return end point of segment in Cartesian coordinates
   */
  Eigen::Vector2d pt_2() const;

  /**
   * @return length of the segment
   */
  double length() const;

  /**
   * @return normal vector at the start of the segment.
   */
  Eigen::Vector2d normalSegmentStart() const;

  /**
   * @return normal vector at the end of the segment.
   */
  Eigen::Vector2d normalSegmentEnd() const;

  /**
   *
   * Normal vector at a specific longitudinal coordinate
   *
   * @param s_local longitudinal coordinate in local coordinate system of
   * segment
   * @return normal vector
   */
  Eigen::Vector2d normal(double s_local) const;

  /**
   *
   * Tangent vector at a specific longitudinal coordinate
   *
   * @param s_local longitudinal coordinate in local coordinate system of
   * segment
   * @return tangent vector
   */
  Eigen::Vector2d tangent(double s_local) const;

 protected:
  /**
   * Transforms a curvilinear point to the Cartesian frame.
   *
   * @param s longitudinal coordinate of point
   * @param l lateral coordinate of point
   * @return the point in the Cartesian frame
   */
  Eigen::Vector2d convertToCartesianCoords(double s, double l) const;

  /**
   * Transforms a point in the global coordinate frame to the curvilinear
   * coordinate frame.
   *
   * @param x global x-coordinate of point
   * @param y global y-coordinate of point
   * @return point in the curvilinear frame
   */
  Eigen::Vector2d convertToCurvilinearCoords(double x, double y) const;

  /**
   * Transforms a rectangle to the Cartesian coordinate frame.
   *
   * @param s_lo_local minimum longitudinal coordinate of the rectangle in the
   * local coordinate frame of the segment
   * @param s_hi_local maximum longitudinal coordinate of the rectangle in the
   * local coordinate frame of the segment
   * @param l_lo minimum lateral coordinate of the rectangle.
   * @param l_hi maximum lateral coordinate of the rectangle
   * @return triangle mesh representing the transformed rectangle in cartesian
   * coordinates.
   */
  std::vector<EigenPolyline> convertRectangleToCartesianCoords(
      double s_lo_local, double s_hi_local, double l_lo, double l_hi) const;

  /**
   * Transforms multiple points in the global coordinate frame to the local
   * coordinate frame of the segment.
   *
   * @param[in] points in the global coordinate frame
   * @param[out] p_local points in the local frame of the segment
   */
  void rotatePointsToLocalFrame(const Eigen::Matrix2Xd points,
                                Eigen::Matrix2Xd &p_local);

  void computeScaledLambdas(const Eigen::Matrix2Xd &p_local,
                            Eigen::RowVectorXd &dividers,
                            Eigen::RowVectorXd &scaled_lambdas);

  Eigen::RowVectorXd computePseudoDistance(const Eigen::RowVectorXd &lambdas,
                                           const Eigen::Matrix2Xd &points);

 private:
  /**
   * Transforms a point in the global coordinate frame to the curvilinear
   * coordinate frame and also returns the corresponding lambda.
   *
   * @param[in] x global x-coordinate of point
   * @param[in] y global y-coordinate of point
   * @param[out] lambda
   * @return point in the curvilinear frame
   */
  Eigen::Vector2d convertToCurvilinearCoords(double x, double y,
                                             double &lambda) const;

  /**
   * Computes the lambda for a given point in the local coordinate system of the
   * segment.
   *
   * @param s_local longitudinal coordinate in local coordinate system of
   * segment
   * @return lambda
   */
  double computeLambda(const Eigen::Vector2d& p_local) const;

  /**
   * Computes the lambda for a given longitudinal coordinate in the local
   * coordinate system of the segment.
   *
   * @param p_local point in the local coordinate system of the segment
   * @return lambda
   */
  double computeLambda(double s) const;

  /**
   * Computes the base point for a given lambda.
   *
   * @param lambda
   * @return base point
   */
  Eigen::Vector2d computeBasePoint(double lambda) const;

  /**
   * Computes the pseudo normal vector for a given base point and point given in
   * the global coordinate frame.
   *
   * @param p_lambda base point
   * @param p point in the global coordinate frame
   * @return pseudo normal
   */
  Eigen::Vector2d computePseudoNormal(const Eigen::Vector2d& p_lambda,
                                      const Eigen::Vector2d& p) const;

  /**
   * Computes the pseudo tangent vector for a given lambda.
   *
   * @param lambda
   * @return pseudo tangent vector in the local coordinate system of the segment
   */
  Eigen::Vector2d computePseudoTangent(double lambda) const;

  /**
   * Computes the pseudo tangent vector for a given lambda.
   *
   * @param lambda
   * @return pseudo tangent vector in the global coordinate system
   */
  Eigen::Vector2d computePseudoTangentGlobal(double lambda) const;

  /**
   * Computes the signed pseudo distance of a point in the local coordinate
   * system of the segment. Points on the left/right side of the segment have a
   * positive/negative pseudo distance.
   *
   * @param pseudo_normal
   * @param p_local point in the local coordinate system of the segment
   * @return signed pseudo distance
   */
  double computeSignedPseudoDistance(const Eigen::Vector2d& pseudo_normal,
                                     const Eigen::Vector2d& p_local) const;

  /**
   * Transforms a point in the global coordinate frame to the local coordinate
   * frame of the segment.
   *
   * @param p point in the global coordinate frame
   * @return point in the local coordinate frame of the segment
   */
  Eigen::Vector2d rotateToLocalFrame(const Eigen::Vector2d& p) const;

  /**
   * Transforms a point in the local coordinate frame of the segment to the
   * global coordinate frame.
   *
   * @param p_local point in the local coordinate system of the segment
   * @return point in the global coordinate frame
   */
  Eigen::Vector2d rotateToGlobalFrame(const Eigen::Vector2d& p_local) const;

 private:
  /// start point of segment
  Eigen::Vector2d pt_1_;
  /// end point of segment
  Eigen::Vector2d pt_2_;
  /// tangent of segment at start point
  Eigen::Vector2d t_1_;
  /// tangent of segment at end point
  Eigen::Vector2d t_2_;
  /// normal of segment at start point
  Eigen::Vector2d n_1_;
  /// normal of segment at end point
  Eigen::Vector2d n_2_;

  /// start point in local segment coordinates
  Eigen::Vector2d pt_1_local_;
  /// end point in local segment coordinates
  Eigen::Vector2d pt_2_local_;
  /// local tangent at pt_1_local_
  Eigen::Vector2d t_1_local_;
  /// local tangent at pt_2_local_
  Eigen::Vector2d t_2_local_;

  /// tangent vector of the line connecting pt_1 and pt_2
  Eigen::Vector2d tangent_;
  /// normal vector to the tangent of the line connecting pt_1 and pt_2
  Eigen::Vector2d normal_;

  /// slopes of tangent vectors: t_1_ = (1, m_1_); t_2_ = (1, m_2_)
  double m_1_;
  double m_2_;

  /// length of the segment
  double length_;
};

}  // namespace geometry

#endif  // SEGMENT_H
