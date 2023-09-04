
#include <geometry/application_settings.h>
#if ENABLE_SERIALIZER
#include <stdlib.h>
#include "geometry/curvilinear_coordinate_system.h"
#include "geometry/serialize/curvilinear_coordinate_system_export.h"
#include "geometry/serialize/public/serialize_public.h"
#include "geometry/serialize/serialize_reg_impl.h"

#include <s11n.net/s11n/algo.hpp>
#include <s11n.net/s11n/micro_api.hpp>
#include <s11n.net/s11n/proxy/std/vector.hpp>
#include <s11n.net/s11n/s11n_debuggering_macros.hpp>
#include <s11n.net/s11n/s11nlite.hpp>

namespace geometry {
namespace serialize {

int serialize(CurvilinearCoordinateSystemConstPtr cs,
              std::ostream &output_stream, const char *format) {
  std::ios_base::fmtflags fmt_flags = output_stream.flags();
  std::streamsize old_precision = output_stream.precision();
  // std::hexfloat(output_stream); // hexadecimal float representation
  output_stream.precision(std::numeric_limits<double>::max_digits10 - 1);
  CurvilinearCoordinateSystemExport *cs_export =
      static_cast<CurvilinearCoordinateSystemExport *>(cs->exportThis());
  if (!cs_export) {
    output_stream.precision(old_precision);
    output_stream.flags(fmt_flags);  // restore flags
    return -1;
  }
  std::shared_ptr<CurvilinearCoordinateSystemExport> cs_export_ptr(cs_export);
  s11nlite::serializer_class(format);
  typedef s11nlite::micro_api<CurvilinearCoordinateSystemExport> CSExportAPI;
  CSExportAPI obj_export_api(format);
  if (obj_export_api.save(*cs_export_ptr, output_stream)) {
    output_stream.precision(old_precision);
    output_stream.flags(fmt_flags);  // restore flags
    return 0;
  }
  output_stream.precision(old_precision);
  output_stream.flags(fmt_flags);  // restore flags
  return -1;
}

int deserialize(CurvilinearCoordinateSystemConstPtr &cs,
                std::istream &input_stream, const char *format) {
  std::ios_base::fmtflags fmt_flags = input_stream.flags();
  // std::hexfloat(input_stream); // hexadecimal float representation
  std::streamsize old_precision = input_stream.precision();

  input_stream.precision(std::numeric_limits<double>::max_digits10 - 1);
  s11nlite::serializer_class(format);
  typedef s11nlite::micro_api<CurvilinearCoordinateSystemExport> CSImportAPI;
  CSImportAPI cs_import_api(format);
  std::shared_ptr<CurvilinearCoordinateSystemExport> loaded_obj_final_ptr(
      cs_import_api.load(input_stream));

  if (!loaded_obj_final_ptr.get()) {
    input_stream.precision(old_precision);
    input_stream.flags(fmt_flags);  // restore flags
    return 1;
  }
  geometry::CurvilinearCoordinateSystem *loaded_cs =
      loaded_obj_final_ptr.get()->loadObject();

  cs = CurvilinearCoordinateSystemConstPtr(loaded_cs);
  input_stream.precision(old_precision);
  input_stream.flags(fmt_flags);  // restore flags
  if (!loaded_cs) {
    return 1;
  } else {
    return 0;
  }
}
}  // namespace serialize
}  // namespace geometry
#endif
