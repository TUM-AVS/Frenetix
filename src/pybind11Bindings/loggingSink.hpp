#include <nanobind/nanobind.h>
// #include <pyerrors.h>

#include <spdlog/spdlog.h>
#include <spdlog/common.h>
#include <spdlog/sinks/base_sink.h>

namespace nb = nanobind;

void setup_logger();
void setup_logger(nb::object logger);