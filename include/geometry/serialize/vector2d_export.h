#pragma once
#include <Eigen/Dense>
#include <istream>

#define S11N_TYPE Eigen::Vector2d
#define S11N_TYPE_NAME "vector2d"
#define S11N_SERIALIZE_FUNCTOR s11n::streamable_type_serialization_proxy
#include <s11n.net/s11n/reg_s11n_traits.hpp>
