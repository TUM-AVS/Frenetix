#pragma once

namespace geometry {
class CurvilinearCoordinateSystem;
namespace serialize {
class ICurvilinearCoordinateSystemExport {
 public:
  ICurvilinearCoordinateSystemExport() {}

  virtual CurvilinearCoordinateSystem *loadObject(void) { return nullptr; };
  virtual ~ICurvilinearCoordinateSystemExport(){};
};
}  // namespace serialize
}  // namespace geometry
