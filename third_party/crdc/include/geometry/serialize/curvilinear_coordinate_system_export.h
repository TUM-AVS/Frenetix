#pragma once
#include <s11n.net/s11n/s11nlite.hpp>  // s11nlite framework
#include "geometry/curvilinear_coordinate_system.h"
#include "geometry/serialize/export_structs/curvilinear_coordinate_system_export_struct.h"
#include "geometry/serialize/icurvilinear_coordinate_system_export.h"

namespace geometry {
namespace serialize {
class CurvilinearCoordinateSystemExport
    : public ICurvilinearCoordinateSystemExport {
 public:
  CurvilinearCoordinateSystemExport(){};
  CurvilinearCoordinateSystemExport(
      const geometry::CurvilinearCoordinateSystem &cs);
  virtual ~CurvilinearCoordinateSystemExport();
  CurvilinearCoordinateSystem *loadObject(void);
  virtual bool operator()(s11nlite::node_type &dest) const;
  virtual bool operator()(const s11nlite::node_type &src);

 protected:
  CurvilinearCoordinateSystemExportStruct m_fields;
  std::vector<Eigen::Vector2d> m_reference_path;
  std::vector<double> m_curvature_vec;
};

}  // namespace serialize

}  // namespace geometry
