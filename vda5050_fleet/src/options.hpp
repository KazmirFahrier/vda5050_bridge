// Copyright 2026 vda5050_bridge contributors
// SPDX-License-Identifier: Apache-2.0
//
// Command line options of the form "--name value", for the two programs of
// this package.

#ifndef OPTIONS_HPP_
#define OPTIONS_HPP_

#include <chrono>
#include <cstdlib>
#include <map>
#include <stdexcept>
#include <string>
#include <utility>

namespace vda5050_fleet
{

class Options
{
public:
  /// `known` maps every option name to its default; "" means no default.
  Options(int argc, char ** argv, std::map<std::string, std::string> known)
  : values_(std::move(known))
  {
    for (int index = 1; index < argc; ++index) {
      const std::string name = argv[index];
      if (name.rfind("--", 0) != 0 || values_.count(name.substr(2)) == 0) {
        throw std::runtime_error("unknown option '" + name + "'");
      }
      if (index + 1 >= argc) {
        throw std::runtime_error("option '" + name + "' needs a value");
      }
      values_[name.substr(2)] = argv[++index];
    }
  }

  const std::string & text(const std::string & name) const {return values_.at(name);}

  /// A length of time given in seconds.
  std::chrono::milliseconds seconds(const std::string & name) const
  {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::duration<double>(number(name)));
  }

  double number(const std::string & name) const
  {
    const std::string & value = values_.at(name);
    char * end = nullptr;
    const double result = std::strtod(value.c_str(), &end);
    if (value.empty() || end == nullptr || *end != '\0') {
      throw std::runtime_error("option '--" + name + "' needs a number, not '" + value + "'");
    }
    return result;
  }

  std::string usage() const
  {
    std::string out;
    for (const auto & [name, value] : values_) {
      out += "  --" + name + (value.empty() ? "" : "  (default: " + value + ")") + "\n";
    }
    return out;
  }

private:
  std::map<std::string, std::string> values_;
};

}  // namespace vda5050_fleet

#endif  // OPTIONS_HPP_
