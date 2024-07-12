#include <mutex>

#include <nanobind/nanobind.h>

#include <spdlog/common.h>
#include <spdlog/sinks/base_sink.h>

#include "loggingSink.hpp"

namespace nb = nanobind;

template<typename Mutex>
class python_logging_sink : public spdlog::sinks::base_sink <Mutex>
{
    nb::object logger;
    nb::object log_record_factory;
protected:
    inline nb::str format_msg(const spdlog::details::log_msg& msg) {
        spdlog::memory_buf_t formatted;
        spdlog::sinks::base_sink<Mutex>::formatter_->format(msg, formatted);
        if (formatted.size() >= 1) {
            // Remove trailing newline
            return nb::str(formatted.data(), formatted.size() - 1);
        } else {
            return nb::str();
        }
    }

    nb::object make_rec(const spdlog::details::log_msg& msg) {
        nb::kwargs args;
        args["name"] = nb::str(msg.logger_name.data(), msg.logger_name.size());
        args["level"] = translate_level(msg.level);
        args["msg"] = format_msg(msg);
        args["exc_info"] = nb::none();
        args["sinfo"] = nb::none();

        args["args"] = nb::tuple();

        if (!msg.source.empty()) {
            args["pathname"] = nb::str(msg.source.filename);
            args["lineno"] = nb::int_(msg.source.line);
            args["func"] = nb::str(msg.source.funcname);
        } else {
            args["pathname"] = "<unknown>";
            args["lineno"] = 0;
            args["func"] = nb::none();
        }

        return log_record_factory(**args);
    }

    void sink_it_(const spdlog::details::log_msg& msg) override
    {
        nb::gil_scoped_acquire gil;
        
        nb::object rec = make_rec(msg);
        
        logger.attr("handle")(std::move(rec));
    }

    void flush_() override 
    {
    }

    using python_level = int;

    static python_level translate_level(spdlog::level::level_enum level) {
        switch (level) {
        case spdlog::level::trace:
            return 5;
        case spdlog::level::debug:
            return 10;
        case spdlog::level::info:
            return 20;
        case spdlog::level::warn:
            return 30;
        case spdlog::level::err:
            return 40;
        case spdlog::level::critical:
            return 50;
        default:
        case spdlog::level::off:
            return 0;
        }
    }

    static nb::module_ logging_module() {
        return nb::module_::import_("logging");
    }

public:

    static nb::object get_logger(std::string name) {
        nb::module_ logging = logging_module();
        nb::str name_str { name.c_str(), name.size() };
        return logging.attr("getLogger")(name_str);
    }

    python_logging_sink(nb::object logger) : logger{logger} {
        nb::module_ logging = logging_module();
        log_record_factory = logging.attr("getLogRecordFactory")();
        
        // Let Python logging take care of formatting (timestamp etc.)
        this->set_pattern("%v");
    }

    python_logging_sink() : python_logging_sink{ get_logger("frenetix") } {
    }
};

// #include <spdlog/details/null_mutex.h>
// using python_logging_sink_st = python_logging_sink<spdlog::details::null_mutex>;
using python_logging_sink_mt = python_logging_sink<std::mutex>;

void setup_logger(nb::object logger) {
    auto python_sink = std::make_shared<python_logging_sink_mt>(logger);
    auto python_forwarding_logger = std::make_shared<spdlog::logger>("default_frenetix_logger", python_sink);
    python_forwarding_logger->set_level(spdlog::level::info);
    spdlog::set_default_logger(python_forwarding_logger);
}

void setup_logger() {
    setup_logger(python_logging_sink_mt::get_logger("frenetix"));
}
