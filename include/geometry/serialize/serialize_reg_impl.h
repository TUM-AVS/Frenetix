#pragma once

#include "geometry/serialize/serialize.h"

#if ENABLE_SERIALIZER

#include <s11n.net/s11n/s11nlite.hpp>

#include "geometry/serialize/curvilinear_coordinate_system_export.h"

#include <s11n.net/s11n/io/serializers.hpp>  // utility code for s11n::io
#include <s11n.net/s11n/s11n_config.hpp>

#define S11N_TYPE geometry::serialize::CurvilinearCoordinateSystemExport
#define S11N_TYPE_NAME "CurvilinearCoordinateSystemExport"
#include <s11n.net/s11n/reg_s11n_traits.hpp>

#endif
