#include "geometry/util.h"
namespace geometry {
namespace util {

int resample_polyline(const RowMatrixXd& polyline, double step,
                      geometry::EigenPolyline& new_polyline) {
  if (polyline.rows() < 2) {
    to_EigenPolyline(polyline, new_polyline);
    return 0;
  }

  new_polyline = geometry::EigenPolyline();
  new_polyline.push_back(polyline.row(0).transpose().eval());
  double current_position = step;
  double current_length = (polyline.row(0) - polyline.row(1)).norm();
  int current_idx = 0;
  while (current_idx < (polyline.rows() - 1)) {
    if (current_position >= current_length) {
      current_position = current_position - current_length;
      current_idx += 1;
      if (current_idx > polyline.rows() - 2) break;
      current_length = (polyline.row(current_idx + 1) - polyline.row(current_idx)).norm();
    } else {
      double rel = current_position / current_length;
      new_polyline.push_back(
          ((1 - rel) * polyline.row(current_idx) + rel * polyline.row(current_idx + 1))
              .eval()
              .transpose()
              .eval());
      current_position += step;
    }
  }

  Eigen::Vector2d last = polyline.bottomRows<1>().transpose();

  // avoid duplicating last point due to precision errors
  if ((last - new_polyline.back()).norm() >= 1e-6) {
      new_polyline.push_back(last.eval());
  }

  return 0;
}

int resample_polyline(const geometry::EigenPolyline& polyline, double step,
                      geometry::EigenPolyline& ret)

{
  RowMatrixXd polyline_converted;
  to_RowMatrixXd(polyline, polyline_converted);
  return resample_polyline(polyline_converted, step, ret);
}

int resample_polyline(const RowMatrixXd& polyline, double step,
                      RowMatrixXd& ret) {
  geometry::EigenPolyline new_polyline;
  int err = resample_polyline(polyline, step, new_polyline);
  to_RowMatrixXd(new_polyline, ret);
  return err;
}

int chaikins_corner_cutting(const RowMatrixXd& polyline, int refinements,
                            RowMatrixXd& ret) {
  RowMatrixXd el2 = polyline.eval();
  for (int cc1 = 0; cc1 < refinements; cc1++) {
    RowMatrixXd el(2 * el2.rows(), el2.cols());

    int j = 0;

    for (int i = 0; i < el2.rows(); ++i) {
      const int k = 2;
      el.middleRows(j, 2) = el2.row(i).colwise().replicate(2);
      j += k;
    }

    RowMatrixXd R = RowMatrixXd::Zero(el.rows(), el.cols());

    R.row(0) = el.row(0);

    for (int cc1 = 2; cc1 < R.rows(); cc1 += 2) {
      R.row(cc1) = el.row(cc1 - 1);
      R.row(cc1 - 1) = el.row(cc1);
    }
    R.row(R.rows() - 1) = el.row(el.rows() - 1);

    el2 = (el * 0.75 + R * 0.25).eval();
  }

  ret = el2;
  return 0;
}

int chaikins_corner_cutting(const geometry::EigenPolyline& polyline,
                            int refinements, geometry::EigenPolyline& ret)

{
  RowMatrixXd polyline_matrix;
  to_RowMatrixXd(polyline, polyline_matrix);
  RowMatrixXd ret_matrix;
  int err = chaikins_corner_cutting(polyline_matrix, refinements, ret_matrix);
  to_EigenPolyline(ret_matrix, ret);
  return err;
}

int to_RowMatrixXd(const geometry::EigenPolyline& polyline, RowMatrixXd& ret) {
  ret = RowMatrixXd(polyline.size(), 2);
  for (int cc1 = 0; cc1 < polyline.size(); cc1++) {
    ret.row(cc1) = polyline[cc1].transpose().eval();
  }
  return 0;
}

int to_EigenPolyline(const RowMatrixXd& polyline,
                     geometry::EigenPolyline& ret) {
  ret = geometry::EigenPolyline();
  for (int cc1 = 0; cc1 < polyline.rows(); cc1++) {
    ret.push_back(polyline.row(cc1).transpose().eval());
  }
  return 0;
}

}  // namespace util
}  // namespace geometry
