#pragma once

#include <boost/log/trivial.hpp>

class Logs {
 public:
  Logs(const std::string& context) : context_(context) {}

  void info(const std::string& message) {
    BOOST_LOG_TRIVIAL(info) << "[" << context_ << "] " << message;
  }

  void error(const std::string& message) {
    BOOST_LOG_TRIVIAL(error) << "[" << context_ << "] " << message;
  }

  // Other log levels (debug, warning, etc.) can be added similarly.

 private:
  std::string context_;
};
