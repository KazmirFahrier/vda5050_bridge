// Copyright 2026 vda5050_bridge contributors
// SPDX-License-Identifier: Apache-2.0

#include "vda5050_core/json.hpp"

#include <cinttypes>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>

namespace vda5050_core
{

namespace
{

/// Recursive-descent parser over a string_view. One instance parses one document.
class Parser
{
public:
  Parser(std::string_view text, std::size_t max_depth)
  : text_(text), max_depth_(max_depth) {}

  Json parse_document()
  {
    skip_whitespace();
    Json value = parse_value(0);
    skip_whitespace();
    if (pos_ != text_.size()) {
      fail("unexpected trailing characters");
    }
    return value;
  }

private:
  [[noreturn]] void fail(const std::string & message) const
  {
    throw JsonParseError(message, pos_);
  }

  bool at_end() const {return pos_ >= text_.size();}
  char peek() const {return at_end() ? '\0' : text_[pos_];}

  void skip_whitespace()
  {
    while (!at_end()) {
      const char c = text_[pos_];
      if (c != ' ' && c != '\t' && c != '\n' && c != '\r') {
        break;
      }
      ++pos_;
    }
  }

  void expect_literal(std::string_view literal)
  {
    if (text_.substr(pos_, literal.size()) != literal) {
      fail("invalid literal");
    }
    pos_ += literal.size();
  }

  Json parse_value(std::size_t depth)
  {
    // The depth limit bounds stack use: a hostile "[[[[[[..." cannot crash the robot.
    if (depth > max_depth_) {
      fail("nesting too deep");
    }
    if (at_end()) {
      fail("unexpected end of input");
    }
    switch (peek()) {
      case '{': return parse_object(depth);
      case '[': return parse_array(depth);
      case '"': return Json(parse_string());
      case 't': expect_literal("true"); return Json(true);
      case 'f': expect_literal("false"); return Json(false);
      case 'n': expect_literal("null"); return Json(nullptr);
      default: return parse_number();
    }
  }

  Json parse_object(std::size_t depth)
  {
    ++pos_;  // '{'
    Json::Object object;
    skip_whitespace();
    if (peek() == '}') {
      ++pos_;
      return Json(std::move(object));
    }
    while (true) {
      skip_whitespace();
      if (peek() != '"') {
        fail("expected string key");
      }
      std::string key = parse_string();
      skip_whitespace();
      if (peek() != ':') {
        fail("expected ':'");
      }
      ++pos_;
      skip_whitespace();
      // Duplicate keys: last one wins (permitted by RFC 8259, and the common behaviour).
      object[std::move(key)] = parse_value(depth + 1);
      skip_whitespace();
      if (peek() == ',') {
        ++pos_;
        continue;
      }
      if (peek() == '}') {
        ++pos_;
        return Json(std::move(object));
      }
      fail("expected ',' or '}'");
    }
  }

  Json parse_array(std::size_t depth)
  {
    ++pos_;  // '['
    Json::Array array;
    skip_whitespace();
    if (peek() == ']') {
      ++pos_;
      return Json(std::move(array));
    }
    while (true) {
      skip_whitespace();
      array.push_back(parse_value(depth + 1));
      skip_whitespace();
      if (peek() == ',') {
        ++pos_;
        continue;
      }
      if (peek() == ']') {
        ++pos_;
        return Json(std::move(array));
      }
      fail("expected ',' or ']'");
    }
  }

  static bool is_digit(char c) {return c >= '0' && c <= '9';}

  Json parse_number()
  {
    // Validate the RFC 8259 grammar by hand; strtod alone would also accept
    // "0x10", "inf", "nan" and leading '+', none of which are JSON.
    const std::size_t start = pos_;
    if (peek() == '-') {
      ++pos_;
    }
    if (peek() == '0') {
      ++pos_;
    } else if (is_digit(peek())) {
      while (is_digit(peek())) {
        ++pos_;
      }
    } else {
      fail("unexpected character");
    }
    if (peek() == '.') {
      ++pos_;
      if (!is_digit(peek())) {
        fail("expected digit after '.'");
      }
      while (is_digit(peek())) {
        ++pos_;
      }
    }
    if (peek() == 'e' || peek() == 'E') {
      ++pos_;
      if (peek() == '+' || peek() == '-') {
        ++pos_;
      }
      if (!is_digit(peek())) {
        fail("expected digit in exponent");
      }
      while (is_digit(peek())) {
        ++pos_;
      }
    }
    const std::string slice(text_.substr(start, pos_ - start));
    const double value = std::strtod(slice.c_str(), nullptr);
    if (!std::isfinite(value)) {
      fail("number out of range");
    }
    return Json(value);
  }

  std::uint32_t parse_hex4()
  {
    if (pos_ + 4 > text_.size()) {
      fail("truncated \\u escape");
    }
    std::uint32_t code = 0;
    for (int i = 0; i < 4; ++i) {
      const char c = text_[pos_++];
      code <<= 4;
      if (c >= '0' && c <= '9') {
        code |= static_cast<std::uint32_t>(c - '0');
      } else if (c >= 'a' && c <= 'f') {
        code |= static_cast<std::uint32_t>(c - 'a' + 10);
      } else if (c >= 'A' && c <= 'F') {
        code |= static_cast<std::uint32_t>(c - 'A' + 10);
      } else {
        --pos_;
        fail("invalid hex digit in \\u escape");
      }
    }
    return code;
  }

  static void append_utf8(std::string & out, std::uint32_t cp)
  {
    if (cp < 0x80) {
      out += static_cast<char>(cp);
    } else if (cp < 0x800) {
      out += static_cast<char>(0xC0 | (cp >> 6));
      out += static_cast<char>(0x80 | (cp & 0x3F));
    } else if (cp < 0x10000) {
      out += static_cast<char>(0xE0 | (cp >> 12));
      out += static_cast<char>(0x80 | ((cp >> 6) & 0x3F));
      out += static_cast<char>(0x80 | (cp & 0x3F));
    } else {
      out += static_cast<char>(0xF0 | (cp >> 18));
      out += static_cast<char>(0x80 | ((cp >> 12) & 0x3F));
      out += static_cast<char>(0x80 | ((cp >> 6) & 0x3F));
      out += static_cast<char>(0x80 | (cp & 0x3F));
    }
  }

  std::string parse_string()
  {
    ++pos_;  // opening quote
    std::string out;
    while (true) {
      if (at_end()) {
        fail("unterminated string");
      }
      const char c = text_[pos_++];
      if (c == '"') {
        return out;
      }
      if (static_cast<unsigned char>(c) < 0x20) {
        --pos_;
        fail("unescaped control character in string");
      }
      if (c != '\\') {
        out += c;
        continue;
      }
      if (at_end()) {
        fail("unterminated escape");
      }
      const char escape = text_[pos_++];
      switch (escape) {
        case '"': out += '"'; break;
        case '\\': out += '\\'; break;
        case '/': out += '/'; break;
        case 'b': out += '\b'; break;
        case 'f': out += '\f'; break;
        case 'n': out += '\n'; break;
        case 'r': out += '\r'; break;
        case 't': out += '\t'; break;
        case 'u': {
            std::uint32_t cp = parse_hex4();
            if (cp >= 0xD800 && cp <= 0xDBFF) {
              // High surrogate: must be followed by a low surrogate to form one code point.
              if (text_.substr(pos_, 2) != "\\u") {
                fail("unpaired surrogate");
              }
              pos_ += 2;
              const std::uint32_t low = parse_hex4();
              if (low < 0xDC00 || low > 0xDFFF) {
                fail("invalid low surrogate");
              }
              cp = 0x10000 + ((cp - 0xD800) << 10) + (low - 0xDC00);
            } else if (cp >= 0xDC00 && cp <= 0xDFFF) {
              fail("unpaired surrogate");
            }
            append_utf8(out, cp);
            break;
          }
        default:
          --pos_;
          fail("invalid escape sequence");
      }
    }
  }

  std::string_view text_;
  std::size_t max_depth_;
  std::size_t pos_{0};
};

void dump_string(const std::string & value, std::string & out)
{
  out += '"';
  for (const char c : value) {
    switch (c) {
      case '"': out += "\\\""; break;
      case '\\': out += "\\\\"; break;
      case '\b': out += "\\b"; break;
      case '\f': out += "\\f"; break;
      case '\n': out += "\\n"; break;
      case '\r': out += "\\r"; break;
      case '\t': out += "\\t"; break;
      default:
        if (static_cast<unsigned char>(c) < 0x20) {
          char buffer[8];
          std::snprintf(buffer, sizeof(buffer), "\\u%04x", static_cast<unsigned>(c));
          out += buffer;
        } else {
          out += c;  // UTF-8 bytes pass through unchanged
        }
    }
  }
  out += '"';
}

void dump_number(double value, std::string & out)
{
  if (!std::isfinite(value)) {
    out += "null";  // JSON has no NaN/Infinity; null is the conventional stand-in
    return;
  }
  char buffer[32];
  // Integers print without a fraction so ids look like 42, not 42.0 or 4.2e+01.
  if (value == std::floor(value) && std::fabs(value) < 9.0e15) {
    std::snprintf(buffer, sizeof(buffer), "%" PRId64, static_cast<std::int64_t>(value));
  } else {
    std::snprintf(buffer, sizeof(buffer), "%.17g", value);  // round-trips any double
  }
  out += buffer;
}

}  // namespace

const Json * Json::find(const std::string & key) const noexcept
{
  const Object * object = std::get_if<Object>(&value_);
  if (object == nullptr) {
    return nullptr;
  }
  const auto it = object->find(key);
  return it == object->end() ? nullptr : &it->second;
}

Json & Json::operator[](const std::string & key)
{
  if (is_null()) {
    value_ = Object{};
  }
  Object * object = std::get_if<Object>(&value_);
  if (object == nullptr) {
    throw JsonTypeError("JSON value is not an object");
  }
  return (*object)[key];
}

void Json::push_back(Json element)
{
  if (is_null()) {
    value_ = Array{};
  }
  Array * array = std::get_if<Array>(&value_);
  if (array == nullptr) {
    throw JsonTypeError("JSON value is not an array");
  }
  array->push_back(std::move(element));
}

Json Json::parse(std::string_view text, std::size_t max_depth)
{
  return Parser(text, max_depth).parse_document();
}

std::string Json::dump() const
{
  std::string out;
  dump_to(out);
  return out;
}

void Json::dump_to(std::string & out) const
{
  if (is_null()) {
    out += "null";
  } else if (is_bool()) {
    out += as_bool() ? "true" : "false";
  } else if (is_number()) {
    dump_number(as_number(), out);
  } else if (is_string()) {
    dump_string(as_string(), out);
  } else if (is_array()) {
    out += '[';
    bool first = true;
    for (const Json & element : as_array()) {
      if (!first) {
        out += ',';
      }
      first = false;
      element.dump_to(out);
    }
    out += ']';
  } else {
    out += '{';
    bool first = true;
    for (const auto & [key, member] : as_object()) {
      if (!first) {
        out += ',';
      }
      first = false;
      dump_string(key, out);
      out += ':';
      member.dump_to(out);
    }
    out += '}';
  }
}

}  // namespace vda5050_core
