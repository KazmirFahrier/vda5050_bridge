// Copyright 2026 vda5050_bridge contributors
// SPDX-License-Identifier: Apache-2.0
//
// Minimal, dependency-free JSON value, parser and writer.
//
// Why this exists: the core library must build and be unit-tested with nothing
// but a C++17 compiler (see docs/design_decisions.md, ADR-003). VDA 5050
// messages arrive from the network, so the parser is strict (RFC 8259) and
// bounded: it rejects trailing data, invalid escapes and excessive nesting
// instead of trying to "do what I mean".

#ifndef VDA5050_CORE__JSON_HPP_
#define VDA5050_CORE__JSON_HPP_

#include <cstddef>
#include <map>
#include <stdexcept>
#include <string>
#include <string_view>
#include <type_traits>
#include <utility>
#include <variant>
#include <vector>

namespace vda5050_core
{

/// Thrown when text is not valid JSON. `offset()` is the byte index of the problem.
class JsonParseError : public std::runtime_error
{
public:
  JsonParseError(const std::string & what, std::size_t offset)
  : std::runtime_error(what + " at byte " + std::to_string(offset)), offset_(offset) {}
  std::size_t offset() const noexcept {return offset_;}

private:
  std::size_t offset_;
};

/// Thrown when a value is accessed as the wrong type (programming error in the caller).
class JsonTypeError : public std::logic_error
{
public:
  using std::logic_error::logic_error;
};

/// An immutable-by-convention JSON value with value semantics.
class Json
{
public:
  using Array = std::vector<Json>;
  using Object = std::map<std::string, Json>;

  /// Default nesting limit for `parse`. Real VDA 5050 messages nest < 10 levels.
  static constexpr std::size_t kDefaultMaxDepth = 64;

  Json() = default;  // null
  Json(std::nullptr_t) {}  // NOLINT(runtime/explicit)
  Json(bool value)
  : value_(value) {}                   // NOLINT(runtime/explicit)
  Json(const char * value)
  : value_(std::string(value)) {}                           // NOLINT(runtime/explicit)
  Json(std::string value)
  : value_(std::move(value)) {}                          // NOLINT(runtime/explicit)
  Json(Array value)
  : value_(std::move(value)) {}                    // NOLINT(runtime/explicit)
  Json(Object value)
  : value_(std::move(value)) {}                     // NOLINT(runtime/explicit)

  /// Any arithmetic type except bool becomes a JSON number (stored as double,
  /// which represents every integer up to 2^53 exactly: enough for uint32 ids).
  template<
    typename T,
    typename = std::enable_if_t<std::is_arithmetic_v<T>&& !std::is_same_v<T, bool>>>
  Json(T value)
  : value_(static_cast<double>(value)) {}                // NOLINT(runtime/explicit)

  bool is_null() const noexcept {return std::holds_alternative<std::nullptr_t>(value_);}
  bool is_bool() const noexcept {return std::holds_alternative<bool>(value_);}
  bool is_number() const noexcept {return std::holds_alternative<double>(value_);}
  bool is_string() const noexcept {return std::holds_alternative<std::string>(value_);}
  bool is_array() const noexcept {return std::holds_alternative<Array>(value_);}
  bool is_object() const noexcept {return std::holds_alternative<Object>(value_);}

  bool as_bool() const {return get<bool>("bool");}
  double as_number() const {return get<double>("number");}
  const std::string & as_string() const {return get<std::string>("string");}
  const Array & as_array() const {return get<Array>("array");}
  const Object & as_object() const {return get<Object>("object");}

  /// Member lookup. Returns nullptr when this is not an object or the key is absent,
  /// so optional fields can be read without exceptions.
  const Json * find(const std::string & key) const noexcept;

  /// Builder access: turns a null value into an object, then inserts/returns the member.
  Json & operator[](const std::string & key);

  /// Builder access: turns a null value into an array, then appends.
  void push_back(Json element);

  bool operator==(const Json & other) const {return value_ == other.value_;}
  bool operator!=(const Json & other) const {return !(*this == other);}

  /// Parse a complete JSON document. Throws JsonParseError.
  static Json parse(std::string_view text, std::size_t max_depth = kDefaultMaxDepth);

  /// Serialise compactly (no whitespace). Object keys are emitted in sorted order,
  /// which makes output deterministic and therefore easy to test and diff.
  std::string dump() const;

private:
  template<typename T>
  const T & get(const char * wanted) const
  {
    if (const T * value = std::get_if<T>(&value_)) {
      return *value;
    }
    throw JsonTypeError(std::string("JSON value is not a ") + wanted);
  }

  void dump_to(std::string & out) const;

  std::variant<std::nullptr_t, bool, double, std::string, Array, Object> value_{nullptr};
};

}  // namespace vda5050_core

#endif  // VDA5050_CORE__JSON_HPP_
