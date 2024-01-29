#include "geometry/curvilinear_coordinate_system.h"

#include "geometry/util.h"

#if ENABLE_SERIALIZER
#include "geometry/serialize/public/serialize_public.h"
#endif

namespace geometry {

CurvilinearCoordinateSystem::CurvilinearCoordinateSystem(
    const EigenPolyline& reference_path, double default_projection_domain_limit,
    double eps, double eps2) {
  this->length_ = 0.0;
  this->segment_longitudinal_coord_.push_back(0.0);
  this->default_projection_domain_limit_ = default_projection_domain_limit;
  this->eps_ = eps;
  this->eps2_ = eps2;

  this->min_curvature_radius_ = std::numeric_limits<double>::quiet_NaN();
  this->max_curvature_radius_ = std::numeric_limits<double>::quiet_NaN();
  this->min_curvature_ = std::numeric_limits<double>::quiet_NaN();
  this->max_curvature_ = std::numeric_limits<double>::quiet_NaN();

  if (reference_path.size() < 3) {
    throw std::invalid_argument(
        "<CurvilinearCoordinateSystem> Reference path must have at least 3 "
        "points.");
  }

  this->reference_path_original_ = reference_path;

  EigenPolyline ref_path;
  if (eps2 != 0) {
    Eigen::Vector2d tangent1 =
        (reference_path[0] - reference_path[1]).normalized();

    Eigen::Vector2d new_point0 = reference_path[0] + tangent1 * 3 * eps2;
    Eigen::Vector2d new_point1 = reference_path[0] + tangent1 * 2 * eps2;

    Eigen::Vector2d new_point2 = reference_path[0] + tangent1 * eps2;

    Eigen::Vector2d tangent2 =
        (reference_path.back() - reference_path[reference_path.size() - 2])
            .normalized();

    Eigen::Vector2d new_point3 = reference_path.back() + tangent2 * eps2;
    Eigen::Vector2d new_point4 = reference_path.back() + tangent2 * 2 * eps2;

    ref_path.push_back(new_point0);
    ref_path.push_back(new_point1);
    ref_path.push_back(new_point2);
    ref_path.insert(ref_path.end(), reference_path.begin(),
                    reference_path.end());
    ref_path.push_back(new_point3);
    ref_path.push_back(new_point4);
  } else {
    ref_path.insert(ref_path.end(), reference_path.begin(),
                    reference_path.end());
  }

  this->reference_path_ = ref_path;

  this->createSegment(ref_path[0], ref_path[1], ref_path[1] - ref_path[0],
                      ref_path[2] - ref_path[0]);
  for (int i = 1; i < ref_path.size() - 2; i++) {
    this->createSegment(ref_path[i], ref_path[i + 1],
                        ref_path[i + 1] - ref_path[i - 1],
                        ref_path[i + 2] - ref_path[i]);
  }
  int last_idx = ref_path.size() - 1;
  this->createSegment(ref_path[last_idx - 1], ref_path[last_idx],
                      ref_path[last_idx] - ref_path[last_idx - 2],
                      ref_path[last_idx] - ref_path[last_idx - 1]);
  this->approximateProjectionDomain(eps);

  this->computeBestProjectionAxisForSegments();
  this->approximateCurvilinearProjectionDomain();
}

EigenPolyline CurvilinearCoordinateSystem::referencePath() const {
  return this->reference_path_;
}

EigenPolyline CurvilinearCoordinateSystem::referencePathOriginal() const {
  return this->reference_path_original_;
}

EigenPolyline CurvilinearCoordinateSystem::projectionDomainBorder() const {
  EigenPolyline border;
  for (auto it = boost::begin(boost::geometry::exterior_ring(this->projection_domain_));
       it != boost::end(boost::geometry::exterior_ring(this->projection_domain_));
       ++it) {
    double x = boost::geometry::get<0>(*it);
    double y = boost::geometry::get<1>(*it);
    border.push_back(Eigen::Vector2d(x, y));
  }

  return border;
}

EigenPolyline CurvilinearCoordinateSystem::curvilinearProjectionDomainBorder()
    const {
  EigenPolyline border;
  for (auto it = boost::begin(boost::geometry::exterior_ring(this->curvilinear_projection_domain_));
       it != boost::end(boost::geometry::exterior_ring(this->curvilinear_projection_domain_));
       ++it) {
    double x = boost::geometry::get<0>(*it);
    double y = boost::geometry::get<1>(*it);
    border.push_back(Eigen::Vector2d(x, y));
  }

  return border;
}

double CurvilinearCoordinateSystem::length() const { return this->length_; }

std::vector<double>
CurvilinearCoordinateSystem::segmentsLongitudinalCoordinates() const {
  return this->segment_longitudinal_coord_;
}

void CurvilinearCoordinateSystem::setCurvature(std::vector<double> curvature) {
  if (curvature.size() != this->segment_longitudinal_coord_.size()) {
    throw std::invalid_argument(
        "<CurvilinearCoordinateSystem/setCurvature> Curvature values must be "
        "given for "
        "each segment.");
  }

  curvature_vec_ = curvature;

  for (int i = 0; i < curvature.size(); i++) {
    this->curvature_[this->segment_longitudinal_coord_[i]] = curvature[i];
  }

  this->max_curvature_ = *std::max_element(curvature.begin(), curvature.end());
  this->min_curvature_ = *std::min_element(curvature.begin(), curvature.end());

  if (fabs(this->max_curvature_) <= 1e-8) {
    this->max_curvature_radius_ = 1e8;
  } else {
    this->max_curvature_radius_ = 1 / fabs(this->max_curvature_);
  }

  if (fabs(this->min_curvature_) <= 1e-8) {
    this->min_curvature_radius_ = 1e8;
  } else {
    this->min_curvature_radius_ = 1 / fabs(this->min_curvature_);
  }
}

std::tuple<double, double> CurvilinearCoordinateSystem::curvatureRange(
    double s_min, double s_max) const {
  // get the first longitudinal position which is lower or equal than s_min
  auto it_min = this->curvature_.upper_bound(s_min);
  if (it_min == this->curvature_.begin()) {
    std::cout << "s_min: " << s_min << std::endl;
    throw CurvilinearProjectionDomainError::longitudinal();
  } else {
    it_min--;
  }

  // get the first longitudinal position which higher or equal than s_max
  auto it_max = this->curvature_.lower_bound(s_max);
  if (it_max == this->curvature_.end()) {
    std::cout << "s_max: " << s_max << std::endl;
    // first and all other longitudinal positions are higher than s_max
    throw CurvilinearProjectionDomainError::longitudinal();
  } else {
    it_max++;
  }

  std::vector<double> curvature_range;
  for (auto it = it_min; it != it_max; ++it) {
    curvature_range.push_back((*it).second);
  }
  std::sort(curvature_range.begin(), curvature_range.end());
  return std::make_tuple(curvature_range.front(), curvature_range.back());
}

double CurvilinearCoordinateSystem::maximumCurvatureRadius() const {
  if (std::isnan(this->max_curvature_radius_)) {
    throw std::invalid_argument(
        "<CurvilinearCoordinateSystem/getMaximumCurvatureRadius> Maximum "
        "curvature radius "
        "must be set first.");
  }
  return this->max_curvature_radius_;
}

double CurvilinearCoordinateSystem::minimumCurvatureRadius() const {
  if (std::isnan(this->min_curvature_radius_)) {
    throw std::invalid_argument(
        "<CurvilinearCoordinateSystem/getMinimumCurvatureRadius> Minimum "
        "curvature radius "
        "must be set first.");
  }
  return this->min_curvature_radius_;
}

double CurvilinearCoordinateSystem::maximumCurvature() const {
  if (std::isnan(this->max_curvature_)) {
    throw std::invalid_argument(
        "<CurvilinearCoordinateSystem/getMaximumCurvature> Maximum curvature "
        "must be set first.");
  }
  return this->max_curvature_;
}

double CurvilinearCoordinateSystem::minimumCurvature() const {
  if (std::isnan(this->min_curvature_)) {
    throw std::invalid_argument(
        "<CurvilinearCoordinateSystem/getMinimumCurvature> Minimum curvature "
        "must be set first.");
  }
  return this->min_curvature_;
}

Eigen::Vector2d CurvilinearCoordinateSystem::normal(double s) const {
  auto idx = this->findSegmentIndex(s);
  auto &segment = this->segment_list_[idx];
  return segment->normal(s - this->segment_longitudinal_coord_[idx]);
}

Eigen::Vector2d CurvilinearCoordinateSystem::tangent(double s) const {
  auto idx = this->findSegmentIndex(s);
  auto &segment = segment_list_[idx];
  return segment->tangent(s - this->segment_longitudinal_coord_[idx]);
}

Eigen::Vector2d CurvilinearCoordinateSystem::convertToCartesianCoords(
    double s, double l) const {
  bool is_in_projection_domain = this->curvilinearPointInProjectionDomain(s, l);
  if (!is_in_projection_domain) {
    throw CurvilinearProjectionDomainError::general();
  }
  int idx = this->findSegmentIndex(s);
  return this->segment_list_[idx]->convertToCartesianCoords(
      s - this->segment_longitudinal_coord_[idx], l);
}

Eigen::Vector2d CurvilinearCoordinateSystem::convertToCurvilinearCoords(
    double x, double y) const {
  int segment_idx = -1;
  return this->convertToCurvilinearCoordsAndGetSegmentIdx(x, y, segment_idx);
}

Eigen::Vector2d
CurvilinearCoordinateSystem::convertToCurvilinearCoordsAndGetSegmentIdx(
    double x, double y, int &segment_idx) const {
  bool is_in_projection_domain = this->cartesianPointInProjectionDomain(x, y);

  if (!is_in_projection_domain) {
    //    std::cout << "Coordinate: " << x << ", " << y << std::endl;
    throw CartesianProjectionDomainError::general();
  }

  std::vector<std::pair<Eigen::Vector2d, int>> candidates;
  double lambda;
  for (int i = 0; i < this->segment_list_.size(); i++) {
    auto &segment = this->segment_list_[i];
    Eigen::Vector2d curvilinear_point =
        segment->convertToCurvilinearCoords(x, y, lambda);
    if (std::isgreaterequal(lambda + 10e-8, 0.0) &&
        std::islessequal(lambda - 10e-8, 1.0)) {
      candidates.push_back(
          std::pair<Eigen::Vector2d, int>(curvilinear_point, i));
    }
  }

  if (candidates.empty()) {
    std::cout << "Coordinate: " << x << ", " << y << std::endl;
    throw CartesianProjectionDomainError::general();
  }

  std::sort(candidates.begin(), candidates.end(),
            [](const std::pair<Eigen::Vector2d, int> &a,
               const std::pair<Eigen::Vector2d, int> &b) {
              return std::abs(a.first(1)) < std::abs(b.first(1));
            });

  segment_idx = candidates[0].second;
  return candidates[0].first +
         Eigen::Vector2d(this->segment_longitudinal_coord_[segment_idx], 0.);
}


EigenPolyline
CurvilinearCoordinateSystem::convertListOfPointsToCurvilinearCoords(
        const EigenPolyline &points, int num_omp_threads) const {
    std::vector<EigenPolyline> points_in(1);
    for (const auto p : points) {
        if (this->cartesianPointInProjectionDomain(p.x(), p.y())) {
            points_in[0].push_back(p);
        }
    }

    std::vector<std::vector<std::tuple<int, double, double>>>
            transformed_coordinates_and_segment_idx;
    transformed_coordinates_and_segment_idx =
            this->convertToCurvilinearCoords(points_in, num_omp_threads);

    EigenPolyline transformed_points;
    if (transformed_coordinates_and_segment_idx.size() == 1) {
        for (const auto p : transformed_coordinates_and_segment_idx[0]) {
            transformed_points.push_back(
                    Eigen::Vector2d(std::get<1>(p), std::get<2>(p)));
        }
    }
    return transformed_points;
}

EigenPolyline CurvilinearCoordinateSystem::convertListOfPointsToCartesianCoords(const EigenPolyline &points,
int num_omp_threads) const {

    // settings for OMP
    omp_set_dynamic(0);
    omp_set_num_threads(num_omp_threads);
    omp_lock_t writelock;
    omp_init_lock(&writelock);

    EigenPolyline reference_path = this->reference_path_;
    // for each point in the polyline
    EigenPolyline group_of_cartesian_points;
    group_of_cartesian_points.resize(points.size());
#pragma omp parallel
    {
#pragma omp for nowait
    for (int point_index = 0; point_index < points.size();
         point_index++) {
	    auto point=points[point_index];
            // get for each point the coordinates
            double s_coordinate = point.x();
            double l_coordinate = point.y();
            bool within_projection_domain = this->curvilinearPointInProjectionDomain(s_coordinate, l_coordinate);
            if (!within_projection_domain) {
                throw CurvilinearProjectionDomainError::general();
            }
            int segment_index = this->findSegmentIndex(s_coordinate);
            Eigen::Vector2d cartesian_coord = this->segment_list_[segment_index]->convertToCartesianCoords(
                    s_coordinate - this->segment_longitudinal_coord_[segment_index], l_coordinate);

            omp_set_lock(&writelock);
            group_of_cartesian_points[point_index] = cartesian_coord;
            omp_unset_lock(&writelock);
        }
    }
    omp_destroy_lock(&writelock);

    return group_of_cartesian_points;
}

EigenPolyline CurvilinearCoordinateSystem::convertRectangleToCartesianCoords(
    double s_lo, double s_hi, double l_lo, double l_hi,
    std::vector<EigenPolyline> &triangle_mesh) const {
  int idx_lo = this->findSegmentIndex(s_lo);
  int idx_hi = this->findSegmentIndex(s_hi);
  triangle_mesh.reserve((idx_hi - idx_lo) * 2);

  for (int idx = idx_lo; idx <= idx_hi; idx++) {
    const auto &segment = this->segment_list_[idx];
    double segment_s_lo =
        std::max(0.0, s_lo - this->segment_longitudinal_coord_[idx]);
    double segment_s_hi = std::min(
        segment->length_, s_hi - this->segment_longitudinal_coord_[idx]);
    auto tmp = segment->convertRectangleToCartesianCoords(
        segment_s_lo, segment_s_hi, l_lo, l_hi);
    triangle_mesh.insert(triangle_mesh.end(), tmp.begin(), tmp.end());
  }
  // Construct vertices of polygon from triangle mesh
  std::vector<Eigen::Vector2d> lower_boundary;
  lower_boundary.reserve(triangle_mesh.size() / 2 + 1);
  std::vector<Eigen::Vector2d> upper_boundary;
  upper_boundary.reserve(triangle_mesh.size() / 2 + 1);

  for (auto it = triangle_mesh.begin(); it != triangle_mesh.end(); it += 2) {
    lower_boundary.push_back((*it)[0]);
    upper_boundary.push_back((*it)[2]);
  }

  EigenPolyline triangle = triangle_mesh.back();
  lower_boundary.push_back(triangle[0]);
  upper_boundary.push_back(triangle[2]);

  EigenPolyline polygon;
  polygon.insert(polygon.end(), upper_boundary.begin(), upper_boundary.end());
  polygon.insert(polygon.end(), lower_boundary.rbegin(), lower_boundary.rend());
  if (!upper_boundary.front().isApprox(lower_boundary.front())) {
    polygon.push_back(upper_boundary.front());
  }

  return polygon;
}

void CurvilinearCoordinateSystem::convertPolygonToCurvilinearCoords(
    const EigenPolyline &polygon,
    std::vector<EigenPolyline> &transformed_polygon) const {
  std::vector<std::vector<EigenPolyline>> transformed_polygons;
  std::vector<std::vector<EigenPolyline>> transformed_polygons_rasterized;

  std::vector<EigenPolyline> polygon_in(1, polygon);
  std::vector<int> groups(1, 0);
  this->convertListOfPolygonsToCurvilinearCoordsAndRasterize(
      polygon_in, groups, 1, 4, transformed_polygons,
      transformed_polygons_rasterized);
  if (transformed_polygons.size() != 0) {
    transformed_polygon = transformed_polygons[0];
  }
}

void CurvilinearCoordinateSystem::
    convertListOfPolygonsToCurvilinearCoordsAndRasterize(
        const std::vector<EigenPolyline> &polygons,
        const std::vector<int> groups_of_polygons, int num_polygon_groups,
        int num_omp_threads,
        std::vector<std::vector<EigenPolyline>> &transformed_polygons,
        std::vector<std::vector<EigenPolyline>>
            &transformed_polygons_rasterized) const {
  omp_set_dynamic(0);
  omp_set_num_threads(num_omp_threads);
  omp_lock_t writelock;
  omp_init_lock(&writelock);

  transformed_polygons.resize(num_polygon_groups);
  transformed_polygons_rasterized.resize(num_polygon_groups);

  // intersect polygons with projection domain
  std::vector<EigenPolyline> clipped_polygon_all;
  std::vector<int> clipped_polygon_groups_all;
  this->determineSubsetsOfMultiPolygonsWithinProjectionDomain(
      polygons, groups_of_polygons, num_omp_threads, clipped_polygon_all,
      clipped_polygon_groups_all);

  // transform all polygons to curvilinear coordinates
  std::vector<std::vector<std::tuple<int, double, double>>>
      poly_curvil_coordinates;
  poly_curvil_coordinates =
      this->convertToCurvilinearCoords(clipped_polygon_all, num_omp_threads);

#pragma omp parallel
  {
    std::vector<std::vector<EigenPolyline>> transformed_polygon_thread_all(
        num_polygon_groups);
    std::vector<std::vector<EigenPolyline>>
        transformed_polygon_rasterized_thread_all(num_polygon_groups);

#pragma omp for nowait
    for (std::vector<EigenPolyline>::const_iterator polygon_it =
             clipped_polygon_all.begin();
         polygon_it < clipped_polygon_all.end(); ++polygon_it) {
      std::vector<EigenPolyline> transformed_polygon_thread;
      std::vector<EigenPolyline> transformed_polygon_rasterized_thread;
      std::set<int> segment_indices_thread;

      int cur_poly_index = polygon_it - clipped_polygon_all.begin();

      // add further points for each transition to a new segment
      EigenPolyline transformed_polygon;
      std::set<int> indices;
      poly_curvil_coordinates[cur_poly_index].push_back(
          poly_curvil_coordinates[cur_poly_index].front());
      bool success = this->addPointsAtSegmentTransition(
          *polygon_it, poly_curvil_coordinates[cur_poly_index],
          transformed_polygon, indices);

      if (success) {
        success = transformed_polygon.size() > 0;
        transformed_polygon_thread.push_back(transformed_polygon);
        segment_indices_thread.insert(indices.begin(), indices.end());
      }

      // approximate transformed polygons with axis-aligned rectangles
      int cur_poly_group = clipped_polygon_groups_all[cur_poly_index];
      if (success && transformed_polygon_thread.size() > 0) {
        this->rasterizeListOfTransformedPolygonsInProjectionDomain(
            transformed_polygon_thread, segment_indices_thread,
            transformed_polygon_rasterized_thread);

        transformed_polygon_thread_all[cur_poly_group].insert(
            transformed_polygon_thread_all[cur_poly_group].end(),
            std::make_move_iterator(transformed_polygon_thread.begin()),
            std::make_move_iterator(transformed_polygon_thread.end()));

        transformed_polygon_rasterized_thread_all[cur_poly_group].insert(
            transformed_polygon_rasterized_thread_all[cur_poly_group].end(),
            std::make_move_iterator(
                transformed_polygon_rasterized_thread.begin()),
            std::make_move_iterator(
                transformed_polygon_rasterized_thread.end()));
      }
    }

    omp_set_lock(&writelock);
    for (int i = 0; i < num_polygon_groups; i++) {
      transformed_polygons[i].insert(
          transformed_polygons[i].end(),
          std::make_move_iterator(transformed_polygon_thread_all[i].begin()),
          std::make_move_iterator(transformed_polygon_thread_all[i].end()));

      transformed_polygons_rasterized[i].insert(
          transformed_polygons_rasterized[i].end(),
          std::make_move_iterator(
              transformed_polygon_rasterized_thread_all[i].begin()),
          std::make_move_iterator(
              transformed_polygon_rasterized_thread_all[i].end()));
    }
    omp_unset_lock(&writelock);
  }

  omp_destroy_lock(&writelock);
}

bool CurvilinearCoordinateSystem::addPointsAtSegmentTransition(
    const EigenPolyline &polygon,
    const std::vector<std::tuple<int, double, double>>
        &curvilinear_coordinates_and_segment_idx,
    EigenPolyline &curvilinear_polygon, std::set<int> &segment_indices) const {
  if (polygon.empty()) {
    return false;
  }

  EigenPolyline polygon_vertices(polygon.begin(), polygon.end());
  int previous_segment_idx;
  int segment_idx;
  Eigen::Vector2d curvilinear_point;

  previous_segment_idx =
      std::get<0>(curvilinear_coordinates_and_segment_idx[0]);
  curvilinear_point =
      Eigen::Vector2d(std::get<1>(curvilinear_coordinates_and_segment_idx[0]),
                      std::get<2>(curvilinear_coordinates_and_segment_idx[0]));
  curvilinear_polygon.push_back(curvilinear_point);
  segment_indices.insert(previous_segment_idx);
  polygon_vertices.push_back(polygon_vertices.front());

  for (int i = 1; i < polygon_vertices.size(); i++) {
    segment_idx = std::get<0>(curvilinear_coordinates_and_segment_idx[i]);
    curvilinear_point = Eigen::Vector2d(
        std::get<1>(curvilinear_coordinates_and_segment_idx[i]),
        std::get<2>(curvilinear_coordinates_and_segment_idx[i]));
    segment_indices.insert(segment_idx);
    // check if new point lies in a different segment than the previous point,
    // if yes, compute intermediate points for each segment in between
    if (segment_idx == previous_segment_idx) {
      curvilinear_polygon.push_back(curvilinear_point);
    } else {
      std::list<int> indices_range =
          this->determineIndicesRange(segment_idx, previous_segment_idx);
      segment_indices.insert(indices_range.begin(), indices_range.end());
      this->interpolatePointsBetweenSegments(polygon_vertices[i - 1],
                                             polygon_vertices[i], indices_range,
                                             curvilinear_polygon);
      if (!curvilinear_polygon.back().isApprox(curvilinear_point, 10e-8)) {
        curvilinear_polygon.push_back(curvilinear_point);
      }
    }
    previous_segment_idx = segment_idx;
  }
  if (curvilinear_polygon.front().isApprox(curvilinear_polygon.back(), 10e-8)) {
    curvilinear_polygon.pop_back();
  }
  return true;
}

std::list<int> CurvilinearCoordinateSystem::determineIndicesRange(
    int segment_idx, int previous_segment_idx) const {
  std::list<int> indices_range;
  if (segment_idx < previous_segment_idx) {
    indices_range.resize(previous_segment_idx - segment_idx + 1);
    std::iota(indices_range.rbegin(), indices_range.rend(), segment_idx);
  } else {
    indices_range.resize(segment_idx - previous_segment_idx + 1);
    std::iota(indices_range.begin(), indices_range.end(), previous_segment_idx);
  }
  return indices_range;
}

void CurvilinearCoordinateSystem::interpolatePointsBetweenSegments(
    const Eigen::Vector2d cartesian_point,
    const Eigen::Vector2d next_cartesian_point,
    const std::list<int> indices_range,
    EigenPolyline &transformed_polygon) const {
  Eigen::Vector2d first_curvilinear_point = transformed_polygon.back();
  EigenPolyline tmp_vertices;
  tmp_vertices.push_back(cartesian_point);
  for (const auto &k : indices_range) {
    Eigen::Vector2d intersection_point;
    const auto &segment = this->segment_list_[k];
    bool intersect = geometry::util::intersectionSegmentSegment(
        segment->pt_2() - (this->default_projection_domain_limit_ + 10.0) *
                              segment->normalSegmentEnd(),
        segment->pt_2() + (this->default_projection_domain_limit_ + 10.0) *
                              segment->normalSegmentEnd(),
        next_cartesian_point, tmp_vertices.back(), intersection_point);

    if (intersect) {
      tmp_vertices.push_back(intersection_point);
      Eigen::Vector2d curvilinear_point = segment->convertToCurvilinearCoords(
          intersection_point[0], intersection_point[1]);
      curvilinear_point +=
          Eigen::Vector2d(this->segment_longitudinal_coord_[k], 0);

      // in the case that the last transformed point lies near the new segment,
      // numerically errors may occur when constructing a polygon, since the new
      // curvilinear_point and the last curvilinear point are almost the same.
      // We skip that point and do not add it to the coordinate list
      if (k == indices_range.front() &&
          first_curvilinear_point.isApprox(curvilinear_point, 10e-8)) {
        transformed_polygon.pop_back();
        transformed_polygon.push_back(curvilinear_point);
      } else {
        transformed_polygon.push_back(curvilinear_point);
      }
    }
  }
}

void CurvilinearCoordinateSystem::
    rasterizeListOfTransformedPolygonsInProjectionDomain(
        const std::vector<EigenPolyline> &transformed_polygons,
        const std::set<int> &segment_indices,
        std::vector<EigenPolyline> &transformed_polygons_rasterized) const {
  for (const auto &polygon : transformed_polygons) {
    std::vector<EigenPolyline> rectangle_list;
    this->rasterizeTransformedPolygonInProjectionDomain(
        polygon, segment_indices, rectangle_list);
    transformed_polygons_rasterized.insert(
        transformed_polygons_rasterized.end(), rectangle_list.begin(),
        rectangle_list.end());
  }
}

void CurvilinearCoordinateSystem::rasterizeTransformedPolygonInProjectionDomain(
    const EigenPolyline &transformed_polygon,
    const std::set<int> &segment_indices,
    std::vector<EigenPolyline> &transformed_polygon_rasterized) const {
  auto createRectangle = [](EigenPolyline points) {
    double x_min = points[0][0];
    double x_max = points[0][0];
    double y_min = points[0][1];
    double y_max = points[0][1];
    for (const auto &p : points) {
      x_min = p[0] < x_min ? p[0] : x_min;
      x_max = p[0] > x_max ? p[0] : x_max;
      y_min = p[1] < y_min ? p[1] : y_min;
      y_max = p[1] > y_max ? p[1] : y_max;
    }
    EigenPolyline rectangle{
        Eigen::Vector2d(x_min, y_min), Eigen::Vector2d(x_max, y_min),
        Eigen::Vector2d(x_max, y_max), Eigen::Vector2d(x_min, y_max)};
    return rectangle;
  };

  std::set<int> segment_indices_extended(segment_indices.begin(),
                                         segment_indices.end());
  segment_indices_extended.insert(*segment_indices.rbegin() + 1);
  if (*segment_indices.begin() > 0) {
    segment_indices_extended.insert(*segment_indices.begin() - 1);
  }

  double last_maximum_s_coord = std::numeric_limits<double>::infinity();
  std::set<int>::iterator it;
  for (it = std::next(segment_indices_extended.begin());
       it != segment_indices_extended.end(); ++it) {
    double s_min = std::min(this->segment_longitudinal_coord_[*std::prev(it)],
                            last_maximum_s_coord) -
                   10e-3;
    double s_max = this->segment_longitudinal_coord_[*it] + 10e-3;

    EigenPolyline points_in_segment;
    if ((it != segment_indices_extended.end()) &&
        (std::next(it) == segment_indices_extended.end())) {
      copy_if(transformed_polygon.begin(), transformed_polygon.end(),
              back_inserter(points_in_segment), [s_min](Eigen::Vector2d p) {
                return std::isgreaterequal(p[0], s_min);
              });
    } else {
      copy_if(transformed_polygon.begin(), transformed_polygon.end(),
              back_inserter(points_in_segment),
              [s_min, s_max](Eigen::Vector2d p) {
                return (std::isgreaterequal(p[0], s_min) &&
                        std::islessequal(p[0], s_max));
              });
    }
    if (!points_in_segment.empty()) {
      EigenPolyline rectangle = createRectangle(points_in_segment);
      transformed_polygon_rasterized.push_back(rectangle);
      last_maximum_s_coord = rectangle[1][0];
    }
  }
}

// Test based on half planes (see https://stackoverflow.com/a/2049593)
static inline double sign(const Eigen::Vector2d &p1, const Eigen::Vector2d &p2, const Eigen::Vector2d &p3) {
    return (p1.x() - p3.x()) * (p2.y() - p3.y()) - (p2.x() - p3.x()) * (p1.y() - p3.y());
}

static inline bool check_point_in_triangle(const Eigen::Vector2d &pt, const Eigen::Vector2d &v1,
                                           const Eigen::Vector2d &v2, const Eigen::Vector2d &v3) {
    double d1 = sign(pt, v1, v2);
    double d2 = sign(pt, v2, v3);
    double d3 = sign(pt, v3, v1);

    bool has_neg = (d1 < 0.0) || (d2 < 0.0) || (d3 < 0.0);
    bool has_pos = (d1 > 0.0) || (d2 > 0.0) || (d3 > 0.0);

    return !(has_neg && has_pos);
}

static inline bool check_point_in_quad(const Eigen::Vector2d &pt, const quad &q) {
    return check_point_in_triangle(pt, q.p1, q.p2, q.p3) || check_point_in_triangle(pt, q.p3, q.p4, q.p1);
}


bool CurvilinearCoordinateSystem::cartesianPointInProjectionDomain(
    double x, double y) const {
    point_type pt(x, y);

    if (!this->curvilinear_projection_domain_quads_valid_) {
        return boost::geometry::covered_by(pt, this->projection_domain_);
    }

    Eigen::Vector2d ept(x, y);
    auto pred = boost::geometry::index::intersects(pt);

    std::vector<quadtree_value_type> result_list;
    this->projection_domain_quads_.query(pred, std::back_inserter(result_list));

    bool quad_res = false;
    for (const auto &result: result_list) {
        const auto& quad = result.second;
        if (check_point_in_quad(ept, quad)) {
            quad_res = true;
            break;
        }
    }

    // Correctness check
#ifndef NDEBUG
    bool orig_res = boost::geometry::covered_by(pt, this->projection_domain_);
    if (orig_res != quad_res) {
        std::cerr << "WARNING! Bug in cartesianPointInProjectionDomain - please report:"
            << std::endl
            << "result mismatch: original is "
            << std::boolalpha << orig_res << ", quad is " << quad_res << std::endl;
        return orig_res;
    }
#endif

    return quad_res;
}

bool CurvilinearCoordinateSystem::curvilinearPointInProjectionDomain(
    double s, double l) const {
    if (!this->curvilinear_projection_domain_quads_valid_) {
        // Shouldn't happen, but better check for it anyways
        throw std::logic_error { "curvilinearPointInProjectionDomain: R*-tree not yet ready" };
    }

    point_type pt(s, l);

    Eigen::Vector2d ept(s, l);
    auto pred = boost::geometry::index::intersects(pt);

    std::vector<quadtree_value_type> result_list;
    this->curvilinear_projection_domain_quads_.query(pred, std::back_inserter(result_list));

    bool quad_res = false;
    for (quadtree_type::const_query_iterator it = this->curvilinear_projection_domain_quads_.qbegin(pred);
         it != this->curvilinear_projection_domain_quads_.qend(); ++it ) {
        if (check_point_in_quad(ept, it->second)) {
            quad_res = true;
            break;
        }
    }

    // Correctness check
#ifndef NDEBUG
    bool orig_res = boost::geometry::within(point_type {s, l}, this->curvilinear_projection_domain_);
    if (orig_res != quad_res) {
        std::cerr << "WARNING! Bug in curvilinearPointInProjectionDomain - please report:"
            << std::endl
            << "result mismatch: original is "
            << std::boolalpha << orig_res << ", quad is " << quad_res << std::endl;

        return orig_res;
    }
#endif

    return quad_res;
}

std::vector<EigenPolyline>
CurvilinearCoordinateSystem::determineSubsetOfPolygonWithinProjectionDomain(
    const EigenPolyline &polygon) const {
  return this->polygonWithinProjectionDomain(polygon, this->projection_domain_);
}

std::vector<EigenPolyline> CurvilinearCoordinateSystem::
    determineSubsetOfPolygonWithinCurvilinearProjectionDomain(
        const EigenPolyline &polygon) const {
  return this->polygonWithinProjectionDomain(
      polygon, this->curvilinear_projection_domain_);
}

void CurvilinearCoordinateSystem::
    determineSubsetsOfMultiPolygonsWithinProjectionDomain(
        const std::vector<EigenPolyline> &polygons,
        const std::vector<int> groups_of_polygons, const int num_omp_threads,
        std::vector<EigenPolyline> &polygons_in_projection_domain,
        std::vector<int> &groups_of_polygons_in_projection_domain) const {
  omp_set_dynamic(0);
  omp_set_num_threads(num_omp_threads);

  omp_lock_t writelock;
  omp_init_lock(&writelock);

#pragma omp parallel
  {
    std::vector<EigenPolyline> polygons_in_projection_domain_thread;
    std::vector<int> groups_of_polygons_in_projection_domain_thread;
#pragma omp for nowait
    for (std::vector<EigenPolyline>::const_iterator polygon_it =
             polygons.begin();
         polygon_it < polygons.end(); ++polygon_it) {
      std::vector<EigenPolyline> polygon_in_proj_domain =
          this->determineSubsetOfPolygonWithinProjectionDomain(*polygon_it);
      std::vector<int> polygon_groups;
      for (int i = 0; i < polygon_in_proj_domain.size(); i++) {
        polygon_groups.push_back(
            groups_of_polygons[polygon_it - polygons.begin()]);
      }

      polygons_in_projection_domain_thread.insert(
          polygons_in_projection_domain_thread.end(),
          std::make_move_iterator(polygon_in_proj_domain.begin()),
          std::make_move_iterator(polygon_in_proj_domain.end()));

      groups_of_polygons_in_projection_domain_thread.insert(
          groups_of_polygons_in_projection_domain_thread.end(),
          std::make_move_iterator(polygon_groups.begin()),
          std::make_move_iterator(polygon_groups.end()));
    }
    omp_set_lock(&writelock);
    polygons_in_projection_domain.insert(
        polygons_in_projection_domain.end(),
        std::make_move_iterator(polygons_in_projection_domain_thread.begin()),
        std::make_move_iterator(polygons_in_projection_domain_thread.end()));
    groups_of_polygons_in_projection_domain.insert(
        groups_of_polygons_in_projection_domain.end(),
        std::make_move_iterator(
            groups_of_polygons_in_projection_domain_thread.begin()),
        std::make_move_iterator(
            groups_of_polygons_in_projection_domain_thread.end()));
    omp_unset_lock(&writelock);
  }
  omp_destroy_lock(&writelock);
}

std::vector<EigenPolyline>
CurvilinearCoordinateSystem::polygonWithinProjectionDomain(
    const EigenPolyline &polygon, const polygon_type &projection_domain) const {
  std::vector<EigenPolyline> polygons_within_projection_domain;

  if (polygon.size() == 0) return polygons_within_projection_domain;

  // over-approximate polygon with axis-aligned rectangle
  polygon_type poly_in;
  double x_min = std::numeric_limits<double>::infinity();
  double x_max = -std::numeric_limits<double>::infinity();
  double y_min = std::numeric_limits<double>::infinity();
  double y_max = -std::numeric_limits<double>::infinity();
  for (const auto &vert : polygon) {
    double x = vert[0];
    double y = vert[1];
    boost::geometry::append(poly_in, point_type(vert[0], vert[1]));
    if (x_min > x) x_min = x;
    if (y_min > y) y_min = y;
    if (x_max < x) x_max = x;
    if (y_max < y) y_max = y;
  }
  polygon_type poly_aabb;
  boost::geometry::append(poly_aabb, point_type(x_min, y_min));
  boost::geometry::append(poly_aabb, point_type(x_min, y_max));
  boost::geometry::append(poly_aabb, point_type(x_max, y_max));
  boost::geometry::append(poly_aabb, point_type(x_max, y_min));
  boost::geometry::append(poly_aabb, point_type(x_min, y_min));

  // fast check with axis-aligned rectangle
  if (boost::geometry::within(poly_aabb, projection_domain)) {
    polygons_within_projection_domain.push_back(polygon);
  } else {
    std::deque<polygon_type> parts_in_projection_domain;
    boost::geometry::intersection(poly_in, projection_domain,
                                  parts_in_projection_domain);
    for (polygon_type const &p : parts_in_projection_domain) {
      EigenPolyline vertices;
      for (auto it = boost::end(boost::geometry::exterior_ring(p)) - 1;
           (it != boost::begin(boost::geometry::exterior_ring(p))); --it) {
        double x = boost::geometry::get<0>(*it);
        double y = boost::geometry::get<1>(*it);
        vertices.emplace_back(x, y);
      }
      polygons_within_projection_domain.push_back(vertices);
    }
  }
  return polygons_within_projection_domain;
}

void CurvilinearCoordinateSystem::createSegment(const Eigen::Vector2d& pt_1,
                                                const Eigen::Vector2d& pt_2,
                                                const Eigen::Vector2d& t_1,
                                                const Eigen::Vector2d& t_2) {
  this->segment_list_.push_back(
      std::make_unique<Segment>(pt_1, pt_2, t_1, t_2));
  this->length_ = this->length_ + this->segment_list_.back()->length();
  this->segment_longitudinal_coord_.push_back(length_);
}

void CurvilinearCoordinateSystem::removeSegment(int ind) {
  // todo: check for correctness

  this->length_ -= this->segment_list_[ind]->length();
  this->segment_list_.erase(this->segment_list_.begin() + ind);
  this->segment_longitudinal_coord_.erase(
      this->segment_longitudinal_coord_.begin() + ind);
}

std::vector<int> CurvilinearCoordinateSystem::findCandidatePointsInSegment(
    int segment_idx,
    const std::vector<std::pair<double, int>> &pair_projected_coord_and_id)
    const {
  double val_min = this->min_best_segm_axis_[segment_idx];
  double val_max = this->max_best_segm_axis_[segment_idx];

  auto low = std::lower_bound(
      pair_projected_coord_and_id.begin(), pair_projected_coord_and_id.end(),
      val_min, [](const std::pair<double, int> &lhs, const double rhs) -> bool {
        return lhs.first < rhs;
      });

  auto high = std::lower_bound(
      pair_projected_coord_and_id.begin(), pair_projected_coord_and_id.end(),
      val_max, [](std::pair<double, int> lhs, const double rhs) -> bool {
        return lhs.first < rhs;
      });
  std::vector<int> candidates_indices;
  for (auto it = low; it < high; it++) {
    int point_id =
        pair_projected_coord_and_id[it - pair_projected_coord_and_id.begin()]
            .second;
    candidates_indices.push_back(point_id);
  }
  return candidates_indices;
}

void CurvilinearCoordinateSystem::determineCurvilinearCoordinatesAndSegmentIdx(
    const std::vector<std::vector<std::tuple<int, int>>>
        &candidate_segments_of_points,
    const std::vector<Eigen::RowVectorXd,
                      Eigen::aligned_allocator<Eigen::RowVectorXd>> &s_coord,
    const std::vector<Eigen::RowVectorXd,
                      Eigen::aligned_allocator<Eigen::RowVectorXd>> &l_coord,
    int num_omp_threads,
    std::vector<std::vector<std::tuple<int, double, double>>>
        &groups_of_curvil_points) const {
  // settings for OMP
  omp_set_dynamic(0);
  omp_set_num_threads(num_omp_threads);

#pragma omp parallel
  {
#pragma omp for nowait
    for (int i = 0; i < groups_of_curvil_points.size(); i++) {
      for (int j = 0; j < groups_of_curvil_points[i].size(); j++) {
        int orig_idx = 0;
        for (int k = 0; k < i; k++) {
          orig_idx += groups_of_curvil_points[k].size();
        }
        orig_idx += j;

        int best_segment = -1;
        int best_idx = -1;
        // there exist more than one valid segment for the point -> take the one
        // with minimal lateral coordinate
        if (candidate_segments_of_points[orig_idx].size() > 1) {
          double signed_distance = std::numeric_limits<double>::infinity();
          for (const auto el : candidate_segments_of_points[orig_idx]) {
            int segment_idx = std::get<0>(el);
            int idx = std::get<1>(el);
            if (std::fabs(l_coord[segment_idx][idx]) < signed_distance) {
              signed_distance = std::fabs(l_coord[segment_idx][idx]);
              best_segment = segment_idx;
              best_idx = idx;
            }
          }
        } else if (candidate_segments_of_points[orig_idx].size() == 1) {
          best_segment = std::get<0>(candidate_segments_of_points[orig_idx][0]);
          best_idx = std::get<1>(candidate_segments_of_points[orig_idx][0]);
        } else {
          throw std::logic_error(
              "<CurvilinearCoordinateSystem/convertToCurvilinearCoords> "
              "Coordinate outside of projection domain.");
        }
        groups_of_curvil_points[i][j] =
            std::make_tuple(best_segment, s_coord[best_segment][best_idx],
                            l_coord[best_segment][best_idx]);
      }
    }
  }
}

void CurvilinearCoordinateSystem::approximateProjectionDomain(double eps) {
  double min_radius, max_radius;
  std::tie(min_radius, max_radius) = this->computeProjectionDomainLimits(eps);
  EigenPolyline projection_domain_border =
      this->computeProjectionDomainBorder(min_radius, max_radius);

  // create boost polygon for projection domain
  this->projection_domain_.clear();
  for (EigenPolyline::iterator it = projection_domain_border.begin();
       it != projection_domain_border.end(); it++) {
    boost::geometry::append(this->projection_domain_,
                            point_type((*it)[0], (*it)[1]));
  }

  for (int i = 0; i < this->upper_projection_domain_border_.size() - 1; i++) {
    const auto& u1 = this->upper_projection_domain_border_.at(i);
    const auto& u2 = this->upper_projection_domain_border_.at(i + 1);

    const auto& l1 = this->lower_projection_domain_border_.at(i);
    const auto& l2 = this->lower_projection_domain_border_.at(i + 1);

    quad q {u1, u2, l2, l1};
    this->projection_domain_quads_.insert(std::make_pair(q.box(), q));

    const auto c_u1 = this->convertToCurvilinearCoords(u1.x(), u1.y());
    const auto c_u2 = this->convertToCurvilinearCoords(u2.x(), u2.y());
    const auto c_l1 = this->convertToCurvilinearCoords(l1.x(), l1.y());
    const auto c_l2 = this->convertToCurvilinearCoords(l2.x(), l2.y());

    quad c_q {c_u1, c_u2, c_l2, c_l1};

    this->curvilinear_projection_domain_quads_.insert(std::make_pair(c_q.box(), c_q));
  }

  this->curvilinear_projection_domain_quads_valid_ = true;
}

void CurvilinearCoordinateSystem::approximateCurvilinearProjectionDomain() {
  std::vector<EigenPolyline> curvilinear_projection_domain;
  this->convertPolygonToCurvilinearCoords(this->projectionDomainBorder(),
                                          curvilinear_projection_domain);

  if (curvilinear_projection_domain.size() != 1) {
    std::cout << curvilinear_projection_domain.size() << std::endl;
    throw std::logic_error(
        "<CurvilinearCoordinateSystem/approximateCurvilinearProjectionDomain> "
        "could not"
        " generate curvilinear projection domain.");
  }

  for (EigenPolyline::reverse_iterator it =
           curvilinear_projection_domain[0].rbegin();
       it != curvilinear_projection_domain[0].rend(); it++) {
    boost::geometry::append(this->curvilinear_projection_domain_,
                            point_type((*it)[0], (*it)[1]));
  }

  if (!curvilinear_projection_domain[0].front().isApprox(
          curvilinear_projection_domain[0].back())) {
    boost::geometry::append(
        this->curvilinear_projection_domain_,
        point_type(curvilinear_projection_domain[0].back()[0],
                   curvilinear_projection_domain[0].back()[1]));
  }
}

std::tuple<double, double>
CurvilinearCoordinateSystem::computeProjectionDomainLimits(double eps) const {
  std::vector<double> intersection_distances;
  // compute intersections between lateral segment vectors
  for (int i = 0; i < this->segment_list_.size() - 1; i++) {
    const auto &segment_1 = this->segment_list_[i];
    Eigen::Vector2d p_1 =
        segment_1->pt_2() - 50.0 * segment_1->normalSegmentEnd();
    Eigen::Vector2d p_2 =
        segment_1->pt_2() + 50.0 * segment_1->normalSegmentEnd();

    const auto &segment_2 = this->segment_list_[i + 1];
    Eigen::Vector2d p_3 =
        segment_2->pt_2() - 50.0 * segment_2->normalSegmentEnd();
    Eigen::Vector2d p_4 =
        segment_2->pt_2() + 50.0 * segment_2->normalSegmentEnd();

    Eigen::Vector2d intersection_point(0., 0.);
    bool intersect = geometry::util::intersectionLineLine(p_1, p_2, p_3, p_4,
                                                          intersection_point);
    if (intersect) {
      Eigen::Vector2d d_1 = intersection_point - segment_1->pt_2();
      Eigen::Vector2d d_2 = intersection_point - segment_2->pt_2();
      double dot_d_1 = (segment_1->normalSegmentEnd())
                           .dot(intersection_point - segment_1->pt_2());
      intersection_distances.push_back(copysign(1.0, dot_d_1) * d_1.norm());
    }
  }

  // get the smallest positive distance and the greatest negative distance
  double min_radius = -this->default_projection_domain_limit_;
  double max_radius = this->default_projection_domain_limit_;
  for (const auto &dis : intersection_distances) {
    if (dis > 0 && dis < max_radius) {
      max_radius = (dis - eps > 0) ? dis - eps : dis;
    } else if (dis < 0 && dis > min_radius) {
      min_radius = (dis + eps < 0) ? dis + eps : dis;
    }
  }

  return std::make_tuple(min_radius, max_radius);
}

EigenPolyline CurvilinearCoordinateSystem::computeProjectionDomainBorder(
    double min_radius, double max_radius) {
  // create upper and lower border of projection domain
  EigenPolyline projection_domain_border;
  for (int i = 1; i < this->segment_list_.size() - 1; i++) {
    const auto &segment = this->segment_list_[i];
    this->upper_projection_domain_border_.push_back(
        segment->pt_2() + max_radius * segment->normalSegmentEnd());
    this->lower_projection_domain_border_.push_back(
        segment->pt_2() + min_radius * segment->normalSegmentEnd());
  }

  // concatenate projection domain border
  projection_domain_border.insert(projection_domain_border.end(),
                                  this->upper_projection_domain_border_.begin(),
                                  this->upper_projection_domain_border_.end());
  projection_domain_border.insert(
      projection_domain_border.end(),
      this->lower_projection_domain_border_.rbegin(),
      this->lower_projection_domain_border_.rend());

  if (!this->upper_projection_domain_border_.front().isApprox(
          this->lower_projection_domain_border_.front())) {
    projection_domain_border.push_back(
        this->upper_projection_domain_border_.front());
  }
  return projection_domain_border;
}

Eigen::VectorXd CurvilinearCoordinateSystem::gradient(
    const Eigen::VectorXd &input) {
  if (input.size() <= 1) return input;
  Eigen::VectorXd res(input.size());
  for (int j = 0; j < input.size(); j++) {
    int j_l = j - 1;
    int j_r = j + 1;
    bool border = false;
    if (j_l < 0) {
      j_l = 0;
      j_r = 1;
      border = true;
    }
    if (j_r >= input.size()) {
      j_r = input.size() - 1;
      j_l = j_r - 1;
      border = true;
    }
    double grad = 0;
    if (border)
      grad = (input[j_r] - input[j_l]);
    else
      grad = (input[j_r] - input[j_l]) / (2.0);
    res[j] = (grad);
  }
  return res;
}

Eigen::VectorXd CurvilinearCoordinateSystem::computeCurvature(
    const geometry::EigenPolyline &polyline) {
  RowMatrixXd polyline_rows;
  geometry::util::to_RowMatrixXd(polyline, polyline_rows);

  Eigen::VectorXd x_d = gradient(polyline_rows.middleCols(0, 1).eval());
  Eigen::VectorXd x_dd = gradient(x_d);
  Eigen::VectorXd y_d = gradient(polyline_rows.middleCols(1, 1).eval());
  Eigen::VectorXd y_dd = gradient(y_d);
  Eigen::VectorXd curvature =
      ((x_d.cwiseProduct(y_dd).eval() - x_dd.cwiseProduct(y_d).eval()).array() /
       ((x_d.array().pow(2) + y_d.array().pow(2)).pow(3. / 2.)))
          .eval();

  return curvature;
}

int CurvilinearCoordinateSystem::computeAndSetCurvature(int digits) {
  auto curvature = computeCurvature(referencePath());
  std::vector<double> curv;
  double precision = pow(10.0, digits);
  for (int cc1 = 0; cc1 < curvature.size(); cc1++) {
    // round value to precision
    curv.push_back(std::round(curvature[cc1] * precision) / precision);
  }
  this->setCurvature(curv);
  return 0;
}

void CurvilinearCoordinateSystem::computeBestProjectionAxisForSegments() {
  for (int i = 0; i < this->segment_list_.size(); i++) {
    auto &segment = this->segment_list_[i];
    // find intersection points of segment normals n_1_ and n_2_ with the upper
    // and lower projection domain border
    EigenPolyline pt(4);
    if (i == 0 || i == 1 || i == this->segment_list_.size() - 1) {
      pt[0] = this->upper_projection_domain_border_
                  [this->upper_projection_domain_border_.size() - 1];
      pt[1] = this->upper_projection_domain_border_[0];  // end of point 1
      pt[2] = this->lower_projection_domain_border_
                  [this->lower_projection_domain_border_.size() - 1];
      pt[3] = this->lower_projection_domain_border_[0];
    } else if (i > 1) {
      pt[0] = this->upper_projection_domain_border_[i - 2];
      pt[1] = this->upper_projection_domain_border_[i - 1];
      pt[2] = this->lower_projection_domain_border_[i - 2];
      pt[3] = this->lower_projection_domain_border_[i - 1];
    }

    // find minimum and maximum longitudinal and lateral coordinates when
    // projecting current segment onto the longitudinal and lateral axis
    // (axis-aligned bounding box of current segment)
    double min_segm_x = std::min(segment->pt_1().x(), segment->pt_2().x());
    double max_segm_x = std::max(segment->pt_1().x(), segment->pt_2().x());
    double min_segm_y = std::min(segment->pt_1().y(), segment->pt_2().y());
    double max_segm_y = std::max(segment->pt_1().y(), segment->pt_2().y());

    for (auto p : pt) {
      if (p.x() < min_segm_x) min_segm_x = p.x();
      if (p.x() > max_segm_x) max_segm_x = p.x();

      if (p.y() < min_segm_y) min_segm_y = p.y();
      if (p.y() > max_segm_y) max_segm_y = p.y();
    }

    std::deque<double> min_segm_axis(4);
    std::deque<double> max_segm_axis(4);
    min_segm_axis[0] = min_segm_x - 1e-8;
    max_segm_axis[0] = max_segm_x + 1e-8;
    min_segm_axis[1] = min_segm_y - 1e-8;
    max_segm_axis[1] = max_segm_y + 1e-8;

    // find minimum and maximum longitudinal and lateral coordinates when
    // projecting current segment onto two diagonal axes (45 degrees) (find
    // orientated bounding box of current segment)
    Eigen::Matrix2d new_axes;
    new_axes << sqrt(2) / 2, sqrt(2) / 2, -sqrt(2) / 2, sqrt(2) / 2;

    Eigen::Vector2d proj_pt1 = util::projectOntoAxes(new_axes, segment->pt_1());
    Eigen::Vector2d proj_pt2 = util::projectOntoAxes(new_axes, segment->pt_2());

    min_segm_x = std::min(proj_pt1.x(), proj_pt2.x());
    max_segm_x = std::max(proj_pt1.x(), proj_pt2.x());
    min_segm_y = std::min(proj_pt1.y(), proj_pt2.y());
    max_segm_y = std::max(proj_pt1.y(), proj_pt2.y());

    for (auto p : pt) {
      Eigen::Vector2d proj_p = util::projectOntoAxes(new_axes, p);
      if (proj_p.x() < min_segm_x) min_segm_x = proj_p.x();
      if (proj_p.x() > max_segm_x) max_segm_x = proj_p.x();

      if (proj_p.y() < min_segm_y) min_segm_y = proj_p.y();
      if (proj_p.y() > max_segm_y) max_segm_y = proj_p.y();
    }

    min_segm_axis[2] = min_segm_x - 1e-8;
    max_segm_axis[2] = max_segm_x + 1e-8;
    min_segm_axis[3] = min_segm_y - 1e-8;
    max_segm_axis[3] = max_segm_y + 1e-8;

    // find axes where the distance to the road boundary within current segment
    // is minimal
    std::vector<double> diff_vec(4);
    diff_vec[0] = (max_segm_axis[0] - min_segm_axis[0]);
    diff_vec[1] = (max_segm_axis[1] - min_segm_axis[1]);
    diff_vec[2] = (max_segm_axis[2] - min_segm_axis[2]);
    diff_vec[3] = (max_segm_axis[3] - min_segm_axis[3]);

    int best_segm_axis_id =
        std::min_element(diff_vec.begin(), diff_vec.end()) - diff_vec.begin();
    this->best_segm_axis_.push_back(ProjectionAxis(best_segm_axis_id));
    this->min_best_segm_axis_.push_back(min_segm_axis[best_segm_axis_id]);
    this->max_best_segm_axis_.push_back(max_segm_axis[best_segm_axis_id]);
  }
}


std::optional<int> CurvilinearCoordinateSystem::findSegmentIndex_Fast(double s) const {
  if ((s < 0) || (s > this->length_)) {
    return std::nullopt;
  }

#ifndef NDEBUG
  bool sorted = std::is_sorted(this->segment_longitudinal_coord_.cbegin(), this->segment_longitudinal_coord_.cend());
  assert(sorted);
#endif

  assert(this->segment_longitudinal_coord_.size() == this->segment_list_.size() + 1);

  // std::lower_bound finds the first segment with a longitudinal coordinate greater or equal s
  // However, we need the last segment with a longitudinal coordinate less than s
  // Therefore, a few special cases need to be handled below
  auto it = std::lower_bound(this->segment_longitudinal_coord_.cbegin(), this->segment_longitudinal_coord_.cend(), s);

  // If lower_bound returns the first segment, then s is lower than all segment longitudinal coordinates
  // and thus outside the projection domain
  if (it == this->segment_longitudinal_coord_.cbegin()) {
    return std::nullopt;
  }

  // Move the iterator to the last segment with a longitudinal coordinate less than s
  it = std::prev(it);

#ifndef NDEBUG
  assert(s >= *it);
  if (std::next(it) != this->segment_longitudinal_coord_.cend()) {
    assert(*it <= *std::next(it));
  }
#endif

  return std::distance(this->segment_longitudinal_coord_.cbegin(), it);
}


std::optional<int> CurvilinearCoordinateSystem::findSegmentIndex_Slow(double s) const {
  if ((s < 0) || (s > this->length_)) {
    return std::nullopt;
  }

  std::optional<int> idx = std::nullopt;
  for (int i = 0; i < this->segment_list_.size(); i++) {
    double s_1 = this->segment_longitudinal_coord_[i];
    double s_2 = this->segment_longitudinal_coord_[i + 1];
    if (std::islessequal(s_1, s) && std::islessequal(s, s_2)) {
      idx = i;
      break;
    }
  }
  return idx;
}


std::optional<int> CurvilinearCoordinateSystem::tryFindSegmentIndex(double s) const {
  auto idx_fast = this->findSegmentIndex_Fast(s);

#ifndef NDEBUG
  auto idx_slow = this->findSegmentIndex_Slow(s);
  assert(idx_slow == idx_fast);
#endif

  return idx_fast;
}


int CurvilinearCoordinateSystem::findSegmentIndex(double s) const {
  try {
      return this->tryFindSegmentIndex(s).value();
  } catch (const std::bad_optional_access& e) {
      throw CurvilinearProjectionDomainError::longitudinal();
  }
}

std::vector<std::vector<std::tuple<int, double, double>>>
CurvilinearCoordinateSystem::convertToCurvilinearCoords(
    const std::vector<EigenPolyline> &groups_of_points,
    int num_omp_threads) const {
  // settings for OMP
  omp_set_dynamic(0);
  omp_set_num_threads(num_omp_threads);
  omp_lock_t writelock;
  omp_init_lock(&writelock);

  // determine number of incoming points
  int number_of_points_in_all_groups = 0;
  for (const auto &points : groups_of_points) {
    number_of_points_in_all_groups += points.size();
  }

  // restructure incoming points
//  [x1 x2 x3 ... xN
//   y1 y2 y3 ... yN]
// We need these to find the candidate points (the closest segment to the point/s)
  Eigen::Matrix2Xd points_in(2, number_of_points_in_all_groups);
  std::vector<std::pair<double, int>> pairs_in_x;
  std::vector<std::pair<double, int>> pairs_in_y;
  std::vector<std::pair<double, int>> pairs_in_x_rotated;
  std::vector<std::pair<double, int>> pairs_in_y_rotated;

  int offset_cols = 0;
  for (const auto &points : groups_of_points) {
    for (const auto &p : points) {
      pairs_in_x.push_back(std::make_pair(p.x(), offset_cols));
      pairs_in_y.push_back(std::make_pair(p.y(), offset_cols));
      points_in.middleCols(offset_cols++, 1) = p;
    }
  }

  // rotate points 45 degree
  Eigen::Matrix2d rot_axis_matr;
//   [ cosA -sinA
//     sinA cosA ]
  rot_axis_matr << sqrt(2) / 2, sqrt(2) / 2, -sqrt(2) / 2, sqrt(2) / 2;
  Eigen::MatrixXd points_in_rotated;
  // Rot(A) * Points_In
  points_in_rotated = rot_axis_matr * points_in.eval();
  for (int i = 0; i < number_of_points_in_all_groups; i++) {
    pairs_in_x_rotated.push_back(std::make_pair(points_in_rotated(0, i), i));
    pairs_in_y_rotated.push_back(std::make_pair(points_in_rotated(1, i), i));
  }

  // sort coordinates of points in increasing order
  std::vector<std::vector<std::pair<double, int>> *> to_sort;
  to_sort.push_back(&pairs_in_x);
  to_sort.push_back(&pairs_in_x_rotated);
  to_sort.push_back(&pairs_in_y);
  to_sort.push_back(&pairs_in_y_rotated);

#pragma omp parallel
  {
#pragma omp for nowait
    for (int sort_index = 0; sort_index < 4; sort_index++) {
      std::sort(to_sort[sort_index]->begin(), to_sort[sort_index]->end());
    }
  }

  std::vector<std::vector<std::tuple<int, int>>> candidate_segments_of_points(
      number_of_points_in_all_groups, std::vector<std::tuple<int, int>>(0));
  std::vector<Eigen::RowVectorXd, Eigen::aligned_allocator<Eigen::RowVectorXd>>
      s_results(this->segment_list_.size());
  std::vector<Eigen::RowVectorXd, Eigen::aligned_allocator<Eigen::RowVectorXd>>
      l_results(this->segment_list_.size());

#pragma omp parallel
  {
#pragma omp for nowait
    for (int segment_index = 0; segment_index < this->segment_list_.size();
         segment_index++) {
      // find points which are potentially within the current segment
      std::vector<int> candidates_indices;
      if (this->best_segm_axis_[segment_index] == ProjectionAxis::X_AXIS) {
        candidates_indices =
            this->findCandidatePointsInSegment(segment_index, pairs_in_x);
      } else if (this->best_segm_axis_[segment_index] ==
                 ProjectionAxis::X_AXIS_ROTATED) {
        candidates_indices = this->findCandidatePointsInSegment(
            segment_index, pairs_in_x_rotated);
      } else if (this->best_segm_axis_[segment_index] ==
                 ProjectionAxis::Y_AXIS) {
        candidates_indices =
            this->findCandidatePointsInSegment(segment_index, pairs_in_y);
      } else if (this->best_segm_axis_[segment_index] ==
                 ProjectionAxis::Y_AXIS_ROTATED) {
        candidates_indices = this->findCandidatePointsInSegment(
            segment_index, pairs_in_y_rotated);
      }
      // get the coordinates of points which might be in the current segment
      Eigen::Matrix2Xd candidate_points_in_segment(2,
                                                   candidates_indices.size());
      int offset_cols = 0;
      for (auto el : candidates_indices) {
        candidate_points_in_segment.middleCols(offset_cols++, 1) =
            points_in.middleCols(el, 1);
      }

      auto &segment = this->segment_list_[segment_index];
      // rotate points to local frame of segment
      Eigen::Matrix2Xd candidate_points_in_segment_local;
      segment->rotatePointsToLocalFrame(candidate_points_in_segment,
                                        candidate_points_in_segment_local);
      // compute scaled lambdas
      Eigen::RowVectorXd scaled_lambdas;
      Eigen::RowVectorXd dividers;
      segment->computeScaledLambdas(candidate_points_in_segment_local, dividers,
                                    scaled_lambdas);
      // check validity of lambdas
      Eigen::RowVectorXd valid_lambdas(candidates_indices.size());
      Eigen::RowVectorXd valid_candidates_y(candidates_indices.size());
      Eigen::Matrix2Xd valid_candidate_points_in_segment(
          2, candidates_indices.size());
      int k = 0;
      int n_cand = 0;
      for (auto orig_ind : candidates_indices) {
        if (dividers(k) > 0.) {
          if ((scaled_lambdas(k) + 10e-8 >= 0.0) &&
              (scaled_lambdas(k) - 10e-8 <= dividers(k))) {
            valid_lambdas(n_cand) = scaled_lambdas(k) / dividers(k);
            valid_candidates_y(n_cand) =
                candidate_points_in_segment_local(1, k);
            omp_set_lock(&writelock);
            candidate_segments_of_points[orig_ind].push_back(
                std::make_tuple(segment_index, n_cand));
            omp_unset_lock(&writelock);
            valid_candidate_points_in_segment.middleCols(n_cand, 1) =
                candidate_points_in_segment.middleCols(k, 1);
            n_cand++;
          }
        }
        k++;
      }
      // for all candidates, compute curvilinear coordinates
      if (n_cand) {
        Eigen::Matrix2Xd candidates_point_in =
            valid_candidate_points_in_segment.block(0, 0, 2, n_cand);
        Eigen::RowVectorXd lambdas = valid_lambdas.block(0, 0, 1, n_cand);
        Eigen::RowVectorXd p_local_y =
            valid_candidates_y.block(0, 0, 1, n_cand);
        // compute signed pseudo distance
        Eigen::RowVectorXd pseudo_distance =
            segment->computePseudoDistance(lambdas, candidates_point_in);
        // compute signed pseudo distance
        Eigen::RowVectorXd signed_distance(lambdas.cols());
        for (int i = 0; i < lambdas.cols(); i++) {
          if (std::isless(p_local_y(i), 0.)) {
            signed_distance(i) = -pseudo_distance(i);
          } else {
            signed_distance(i) = pseudo_distance(i);
          }
        }
        // compute longitudinal coordinate
        Eigen::RowVectorXd lengths =
            Eigen::RowVectorXd::Constant(n_cand, segment->length_);
        Eigen::RowVectorXd s_offset = Eigen::RowVectorXd::Constant(
            n_cand, this->segment_longitudinal_coord_[segment_index]);
        Eigen::RowVectorXd lon_coord = lengths.cwiseProduct(lambdas) + s_offset;

        s_results[segment_index] = lon_coord.eval();
        l_results[segment_index] = signed_distance.eval();
      }
    }
  }

  // segment index, longitudinal coordinate, lateral coordinate
  std::vector<std::vector<std::tuple<int, double, double>>>
      groups_of_curvil_points(groups_of_points.size());
  for (int i = 0; i < groups_of_points.size(); i++) {
    groups_of_curvil_points[i].resize(groups_of_points[i].size());
  }
  this->determineCurvilinearCoordinatesAndSegmentIdx(
      candidate_segments_of_points, s_results, l_results, num_omp_threads,
      groups_of_curvil_points);
  omp_destroy_lock(&writelock);
  return groups_of_curvil_points;
}


#if ENABLE_SERIALIZER

int CurvilinearCoordinateSystem::serialize(std::ostream &output_stream) const {
  return serialize::serialize(
      std::static_pointer_cast<const CurvilinearCoordinateSystem>(
          this->shared_from_this()),
      output_stream);
}

CurvilinearCoordinateSystemConstPtr CurvilinearCoordinateSystem::deserialize(
    std::istream &input_stream) {
  CurvilinearCoordinateSystemConstPtr ret;
  if (!serialize::deserialize(ret, input_stream)) {
    return ret;
  } else
    return CurvilinearCoordinateSystemConstPtr(0);
}

namespace serialize {
ICurvilinearCoordinateSystemExport *exportObject(
    const geometry::CurvilinearCoordinateSystem &);
}

serialize::ICurvilinearCoordinateSystemExport *
CurvilinearCoordinateSystem::exportThis(void) const {
  return serialize::exportObject(*this);
}
#endif

}  // namespace geometry
