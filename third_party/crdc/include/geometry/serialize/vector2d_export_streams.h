#pragma once

// warning: to be included only with a serialize_.cc file: unsafe streaming
// operator redefinition for a generic version see also:
// https://stackoverflow.com/questions/20786220/eigen-library-initialize-matrix-with-data-from-file-or-existing-stdvector

#include <Eigen/Dense>
#include <istream>

inline std::istream &operator>>(std::istream &in, Eigen::Vector2d &vec) {
  in >> vec[0];
  in >> vec[1];

  return in;
}

inline std::ostream &operator<<(std::ostream &out, const Eigen::Vector2d &vec) {
  out << vec[0];
  out << " ";
  out << vec[1];
  return out;
}
