#include <math.h>

#include "geometry/segment.h"

namespace geometry {

Segment::Segment(const Eigen::Vector2d& pt_1, const Eigen::Vector2d& pt_2,
                 const Eigen::Vector2d& t_1,  const Eigen::Vector2d& t_2) {
  this->pt_1_ = pt_1;
  this->pt_2_ = pt_2;
  this->t_1_ = t_1.normalized();
  this->t_2_ = t_2.normalized();
  this->n_1_ = Eigen::Vector2d(-this->t_1_[1], this->t_1_[0]);
  this->n_2_ = Eigen::Vector2d(-this->t_2_[1], this->t_2_[0]);

  this->tangent_ = (this->pt_2_ - this->pt_1_).normalized();
  this->normal_ = Eigen::Vector2d(-this->tangent_[1], this->tangent_[0]);
  this->length_ = (this->pt_1_ - this->pt_2_).norm();

  this->pt_1_local_ = Eigen::Vector2d(0., 0.);
  this->pt_2_local_ = Eigen::Vector2d(0., this->length_);
  this->t_1_local_ = this->rotateToLocalFrame(t_1).normalized();
  this->m_1_ = this->t_1_local_[1] / this->t_1_local_[0];
  this->t_2_local_ = this->rotateToLocalFrame(t_2).normalized();
  this->m_2_ = this->t_2_local_[1] / this->t_2_local_[0];
};

Eigen::Vector2d Segment::pt_1() const { return this->pt_1_; }

Eigen::Vector2d Segment::pt_2() const { return this->pt_2_; }

double Segment::length() const { return this->length_; }

Eigen::Vector2d Segment::convertToCartesianCoords(double s, double l) const {
  double lambda = s / this->length_;
  Eigen::Vector2d pseudo_tangent = this->computePseudoTangent(lambda);
  Eigen::Vector2d p_local =
      Eigen::Vector2d(s, 0) +
      l * Eigen::Vector2d(-pseudo_tangent[1], pseudo_tangent[0]);
  Eigen::Vector2d p = this->rotateToGlobalFrame(p_local);
  return p + this->pt_1_;
}

Eigen::Vector2d Segment::convertToCurvilinearCoords(double x, double y) const {
  double lambda = 0.;
  return this->convertToCurvilinearCoords(x, y, lambda);
}

Eigen::Vector2d Segment::convertToCurvilinearCoords(double x, double y,
                                                    double &lambda) const {
  Eigen::Vector2d p(x, y);
  Eigen::Vector2d p_local = this->rotateToLocalFrame(p - this->pt_1_);
  lambda = this->computeLambda(p_local);
  Eigen::Vector2d p_lambda = this->computeBasePoint(lambda);
  Eigen::Vector2d pseudo_normal = this->computePseudoNormal(p_lambda, p);
  double pseudo_distance =
      this->computeSignedPseudoDistance(pseudo_normal, p_local);
  return Eigen::Vector2d(lambda * this->length_, pseudo_distance);
}

std::vector<EigenPolyline> Segment::convertRectangleToCartesianCoords(
    double s_lo_local, double s_hi_local, double l_lo, double l_hi) const {
  std::vector<EigenPolyline> triangle_mesh;

  Eigen::Vector2d pt_1 = this->convertToCartesianCoords(s_lo_local, l_lo);
  Eigen::Vector2d pt_2 = this->convertToCartesianCoords(s_hi_local, l_lo);
  Eigen::Vector2d pt_3 = this->convertToCartesianCoords(s_lo_local, l_hi);
  Eigen::Vector2d pt_4 = this->convertToCartesianCoords(s_hi_local, l_hi);

  EigenPolyline triangle_1 = {pt_1, pt_2, pt_3};
  EigenPolyline triangle_2 = {pt_2, pt_3, pt_4};
  triangle_mesh.push_back(triangle_1);
  triangle_mesh.push_back(triangle_2);
  return triangle_mesh;
}

Eigen::Vector2d Segment::normalSegmentStart() const { return this->n_1_; }

Eigen::Vector2d Segment::normalSegmentEnd() const { return this->n_2_; }

Eigen::Vector2d Segment::normal(double s_local) const {
  double lambda = this->computeLambda(s_local);
  Eigen::Vector2d pseudo_tangent = this->computePseudoTangentGlobal(lambda);
  Eigen::Vector2d pseudo_normal =
      Eigen::Vector2d(-pseudo_tangent[1], pseudo_tangent[0]);
  return pseudo_normal;
}

Eigen::Vector2d Segment::tangent(double s_local) const {
  double lambda = this->computeLambda(s_local);
  Eigen::Vector2d pseudo_tangent = this->computePseudoTangentGlobal(lambda);
  return pseudo_tangent;
}

void Segment::rotatePointsToLocalFrame(const Eigen::Matrix2Xd points,
                                       Eigen::Matrix2Xd &p_local) {
  Eigen::Matrix2d R;
  R << this->tangent_[0], this->tangent_[1], this->normal_[0], this->normal_[1];

  Eigen::Matrix2Xd pt_1_matr(2, points.cols());
  pt_1_matr = this->pt_1_.replicate(1, points.cols());

  Eigen::Matrix2Xd pt_1_matr_tmp(2, points.cols());
  pt_1_matr_tmp = points - pt_1_matr;

  p_local = R * (pt_1_matr_tmp);
}

void Segment::computeScaledLambdas(const Eigen::Matrix2Xd &p_local,
                                   Eigen::RowVectorXd &dividers,
                                   Eigen::RowVectorXd &scaled_lambdas) {
  Eigen::RowVectorXd length_vec =
      Eigen::RowVectorXd::Constant(p_local.cols(), this->length_);
  dividers =
      length_vec - ((this->m_2_ - this->m_1_) * p_local.middleRows(1, 1));
  scaled_lambdas =
      (p_local.middleRows(0, 1) + p_local.middleRows(1, 1) * this->m_1_);
}

Eigen::RowVectorXd Segment::computePseudoDistance(
    const Eigen::RowVectorXd &lambdas, const Eigen::Matrix2Xd &points) {
  Eigen::Matrix2Xd tmp1(2, lambdas.cols());
  Eigen::Matrix2Xd diff_tmp =
      (this->pt_2_ - this->pt_1_).replicate(1, lambdas.cols());
  tmp1 = diff_tmp.cwiseProduct(lambdas.replicate(2, 1));
  Eigen::Matrix2Xd tmp2 = this->pt_1_.replicate(1, lambdas.cols());
  Eigen::Matrix2Xd p_lambdas = tmp1 + tmp2;
  Eigen::Matrix2Xd pseudo_normals = points - p_lambdas;
  Eigen::RowVectorXd pseudonorm_x = pseudo_normals.middleRows(0, 1);
  Eigen::RowVectorXd pseudonorm_y = pseudo_normals.middleRows(1, 1);
  Eigen::RowVectorXd distance = (pseudonorm_x.cwiseProduct(pseudonorm_x) +
                                 pseudonorm_y.cwiseProduct(pseudonorm_y))
                                    .cwiseSqrt();
  return distance;
}

double Segment::computeLambda(double s) const { return s / this->length_; }

double Segment::computeLambda(const Eigen::Vector2d& p_local) const {
  double lambda = -1;
  double devider = this->length_ - p_local[1] * (this->m_2_ - this->m_1_);
  if (std::isgreater(std::abs(devider), 0.)) {
    lambda = (p_local[0] + p_local[1] * m_1_) / devider;
  }
  return lambda;
}

Eigen::Vector2d Segment::computeBasePoint(double lambda) const {
  return lambda * this->pt_2_ + (1. - lambda) * this->pt_1_;
}

Eigen::Vector2d Segment::computePseudoNormal(const Eigen::Vector2d& p_lambda,
                                             const Eigen::Vector2d& p) const {
  return p - p_lambda;
}

Eigen::Vector2d Segment::computePseudoTangent(double lambda) const {
  return (lambda * this->t_2_local_ + (1. - lambda) * this->t_1_local_)
      .normalized();
}

Eigen::Vector2d Segment::computePseudoTangentGlobal(double lambda) const {
  return (lambda * this->t_2_ + (1. - lambda) * this->t_1_).normalized();
}

double Segment::computeSignedPseudoDistance(const Eigen::Vector2d& pseudo_normal,
                                            const Eigen::Vector2d& p_local) const {
  double pseudo_distance = pseudo_normal.norm();
  if (std::isless(p_local[1], 0.)) {
    pseudo_distance = -pseudo_distance;
  }
  return pseudo_distance;
}

Eigen::Vector2d Segment::rotateToLocalFrame(const Eigen::Vector2d& p) const {
  Eigen::Matrix2d R;
  R << this->tangent_[0], this->tangent_[1], this->normal_[0], this->normal_[1];
  return R * p;
}

Eigen::Vector2d Segment::rotateToGlobalFrame(const Eigen::Vector2d& p_local) const {
  Eigen::Matrix2d R;
  R << this->tangent_[0], this->normal_[0], this->tangent_[1], this->normal_[1];
  return R * p_local;
}

}  // namespace geometry
