#include "geometry/application_settings.h"
#if ENABLE_SERIALIZER

#include <Eigen/Dense>
#include <istream>

#include "geometry/serialize/vector2d_export_streams.h" // redefines the stream operators for the module

#include "geometry/serialize/serialize.h"

#include "geometry/curvilinear_coordinate_system.h"
#include "geometry/serialize/basic_types.h"
#include "geometry/serialize/icurvilinear_coordinate_system_export.h"
#include "geometry/serialize/vector2d_export.h"
#include <s11n.net/s11n/proxy/std/vector.hpp>
#include <s11n.net/s11n/s11nlite.hpp>

namespace geometry {
namespace serialize {
ICurvilinearCoordinateSystemExport *exportObject(
    const geometry::CurvilinearCoordinateSystem &cs) {
  return new CurvilinearCoordinateSystemExport(cs);
}
CurvilinearCoordinateSystemExport::CurvilinearCoordinateSystemExport(
    const geometry::CurvilinearCoordinateSystem &cs) {
  m_fields.default_projection_domain_limit = cs.defaultProjectionDomainLimit();
  m_fields.eps = cs.eps();
  m_fields.eps2 = cs.eps2();

  for (auto el : cs.referencePathOriginal()) {
    m_reference_path.push_back(el);
  }

  m_curvature_vec = cs.curvatureVector();
}

CurvilinearCoordinateSystemExport::~CurvilinearCoordinateSystemExport() {}

bool CurvilinearCoordinateSystemExport::operator()(
    s11nlite::node_type &dest) const {
  typedef s11nlite::node_traits TR;
  TR::class_name(dest, "CurvilinearCoordinateSystemExport");
  TR::set(dest, "default_projection_domain_limit",
          m_fields.default_projection_domain_limit);
  TR::set(dest, "eps", m_fields.eps);
  TR::set(dest, "eps2", m_fields.eps2);

  bool res = true;
  res = res && s11n::list::serialize_list(dest, "ref_path", m_reference_path);
  res =
      res && s11n::list::serialize_list(dest, "curvature_vec", m_curvature_vec);

  return res;
}
bool CurvilinearCoordinateSystemExport::operator()(
    const s11nlite::node_type &src) {
  typedef s11nlite::node_traits TR;
  m_fields.default_projection_domain_limit =
      TR::get(src, "default_projection_domain_limit", double(0));
  m_fields.eps = TR::get(src, "eps", double(0));
  m_fields.eps2 = TR::get(src, "eps2", double(0));

  bool res = true;
  res = res &&
        s11n::list::deserialize_list(src, "ref_path", this->m_reference_path);

  res = res && s11n::list::deserialize_list(src, "curvature_vec",
                                            this->m_curvature_vec);

  return res;
}

CurvilinearCoordinateSystem *CurvilinearCoordinateSystemExport::loadObject(
    void) {
  EigenPolyline ref_path;
  for (auto el : m_reference_path) {
    ref_path.push_back(el);
  }

  CurvilinearCoordinateSystem *cs = new CurvilinearCoordinateSystem(
      ref_path, m_fields.default_projection_domain_limit, m_fields.eps,
      m_fields.eps2);

  if (m_curvature_vec.size() > 0) {
    cs->setCurvature(m_curvature_vec);
  }

  return cs;
}
}  // namespace serialize
}  // namespace geometry
#endif
