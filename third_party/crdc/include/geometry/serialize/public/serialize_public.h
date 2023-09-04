#pragma once

#include <iostream>
#include "geometry/curvilinear_coordinate_system.h"

namespace geometry {
namespace serialize {

int serialize(CurvilinearCoordinateSystemConstPtr collision_object,
              std::ostream &output_stream,
              const char *format = SERIALIZER_DEFAULT_FORMAT);
int deserialize(CurvilinearCoordinateSystemConstPtr &collision_object,
                std::istream &input_stream,
                const char *format = SERIALIZER_DEFAULT_FORMAT);

}  // namespace serialize
}  // namespace geometry
