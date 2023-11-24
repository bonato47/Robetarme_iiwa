#pragma once

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/log/attributes/clock.hpp>
#include <boost/log/core.hpp>
#include <boost/log/expressions.hpp>
#include <boost/log/sinks.hpp>
#include <boost/log/sources/logger.hpp>
#include <boost/log/sources/severity_logger.hpp>
#include <boost/log/trivial.hpp>
#include <boost/log/utility/setup/common_attributes.hpp>
#include <boost/log/utility/setup/console.hpp>
#include <boost/log/utility/setup/file.hpp>
#include <iostream>
#include <locale>
#include <string>

namespace logging = boost::log;
namespace trivial = logging::trivial;
namespace expr = logging::expressions;
namespace sinks = boost::log::sinks;
namespace attrs = boost::log::attributes;
namespace keys = logging::keywords;

class Logs {
 public:
  Logs(
      const std::string& context, trivial::severity_level log_level,
      bool log_to_file, const std::string& filename
  ) :
      context_(context),
      log_level_(log_level),
      log_to_file_(log_to_file),
      filename_(filename) {
    init();
  }

  void trace(const std::string& message) {
    BOOST_LOG_TRIVIAL(trace) << "[" << context_ << "] " << message;
  }

  void debug(const std::string& message) {
    BOOST_LOG_TRIVIAL(debug) << "[" << context_ << "] " << message;
  }

  void info(const std::string& message) {
    BOOST_LOG_TRIVIAL(info) << "[" << context_ << "] " << message;
  }

  void warning(const std::string& message) {
    BOOST_LOG_TRIVIAL(warning) << "[" << context_ << "] " << message;
  }

  void error(const std::string& message) {
    BOOST_LOG_TRIVIAL(error) << "[" << context_ << "] " << message;
  }

  void fatal(const std::string& message) {
    BOOST_LOG_TRIVIAL(fatal) << "[" << context_ << "] " << message;
  }

  // Add other log levels (debug, warning, etc.) as needed.

 private:
  std::string context_;
  trivial::severity_level log_level_;
  bool log_to_file_;
  std::string filename_;

  void init() {
    logging::core::get()->set_filter(trivial::severity >= log_level_);
    logging::core::get()->add_global_attribute(
        "TimeStamp", attrs::local_clock()
    );

    // Create a console log sink
    auto console_sink = logging::add_console_log(std::cout);
    customize_color(console_sink);

    // Optionally, log to a file if log_to_file_ is true.
    if (log_to_file_) {
      logging::add_file_log(
          keys::file_name = filename_, keys::auto_flush = true,
          keys::format = "[%TimeStamp%] [%Severity%] %Message%"
      );
    }
  }

  void customize_color(
      boost::shared_ptr<sinks::synchronous_sink<sinks::text_ostream_backend>>
          console_sink
  ) {
    console_sink->set_formatter([&](const logging::record_view& rec,
                                    logging::formatting_ostream& strm) {
      // Extract the severity level from the log record
      auto severity = rec[trivial::severity];

      // Customize colors based on severity
      if (severity == trivial::trace) {
        strm << "\033[1;37m";  // White color for TRACE messages
      } else if (severity == trivial::debug) {
        strm << "\033[1;32m";  // Cyan color for DEBUG messages
      } else if (severity == trivial::info) {
        strm << "\033[1;36m";  // Green color for INFO messages
      } else if (severity == trivial::warning) {
        strm << "\033[1;33m";  // Yellow color for WARNING messages
      } else if (severity == trivial::error) {
        strm << "\033[1;31m";  // Red color for ERROR messages
      } else if (severity == trivial::fatal) {
        strm << "\033[1;35m";  // Magenta color for FATAL messages
      }

      // Add severity level and message
      strm << logging::extract<boost::posix_time::ptime>("TimeStamp", rec)
           << " [" << severity << "] " << rec[expr::smessage];

      // Reset text color
      strm << "\033[0m";
    });
  }
};