// Copyright 2026 vda5050_bridge contributors
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>

#include <string>

#include "vda5050_core/json.hpp"

using vda5050_core::Json;
using vda5050_core::JsonParseError;
using vda5050_core::JsonTypeError;

TEST(Json, ParsesScalars)
{
  EXPECT_TRUE(Json::parse("null").is_null());
  EXPECT_TRUE(Json::parse("true").as_bool());
  EXPECT_FALSE(Json::parse("false").as_bool());
  EXPECT_DOUBLE_EQ(Json::parse("42").as_number(), 42.0);
  EXPECT_DOUBLE_EQ(Json::parse("-0.5").as_number(), -0.5);
  EXPECT_DOUBLE_EQ(Json::parse("1.5e3").as_number(), 1500.0);
  EXPECT_EQ(Json::parse("\"hi\"").as_string(), "hi");
}

TEST(Json, ParsesNestedStructuresAndIgnoresWhitespace)
{
  const Json value = Json::parse(" { \"a\" : [ 1 , { \"b\" : null } ] ,\n\t\"c\" : \"d\" } ");
  ASSERT_TRUE(value.is_object());
  const Json * a = value.find("a");
  ASSERT_NE(a, nullptr);
  ASSERT_EQ(a->as_array().size(), 2u);
  EXPECT_DOUBLE_EQ(a->as_array()[0].as_number(), 1.0);
  EXPECT_TRUE(a->as_array()[1].find("b")->is_null());
  EXPECT_EQ(value.find("c")->as_string(), "d");
  EXPECT_EQ(value.find("missing"), nullptr);
}

TEST(Json, DecodesStringEscapes)
{
  EXPECT_EQ(Json::parse(R"("a\"b\\c\/d\n\t")").as_string(), "a\"b\\c/d\n\t");
  // \u00e9 is U+00E9 (2 UTF-8 bytes); the surrogate pair is U+1F600 (4 bytes).
  EXPECT_EQ(Json::parse(R"("\u00e9")").as_string(), "\xC3\xA9");
  EXPECT_EQ(Json::parse(R"("\ud83d\ude00")").as_string(), "\xF0\x9F\x98\x80");
}

TEST(Json, RejectsMalformedInput)
{
  EXPECT_THROW(Json::parse(""), JsonParseError);
  EXPECT_THROW(Json::parse("{"), JsonParseError);
  EXPECT_THROW(Json::parse("[1,]"), JsonParseError);
  EXPECT_THROW(Json::parse("{\"a\":1,}"), JsonParseError);
  EXPECT_THROW(Json::parse("{a:1}"), JsonParseError);
  EXPECT_THROW(Json::parse("tru"), JsonParseError);
  EXPECT_THROW(Json::parse("\"unterminated"), JsonParseError);
  EXPECT_THROW(Json::parse("\"bad \\x escape\""), JsonParseError);
  EXPECT_THROW(Json::parse("\"\\ud83d\""), JsonParseError);  // lone surrogate
  EXPECT_THROW(Json::parse("\"line\nbreak\""), JsonParseError);  // raw control character
  EXPECT_THROW(Json::parse("1 2"), JsonParseError);  // trailing data
}

TEST(Json, RejectsNumbersThatAreNotJson)
{
  EXPECT_THROW(Json::parse("+1"), JsonParseError);
  EXPECT_THROW(Json::parse("01"), JsonParseError);
  EXPECT_THROW(Json::parse("1."), JsonParseError);
  EXPECT_THROW(Json::parse(".5"), JsonParseError);
  EXPECT_THROW(Json::parse("1e"), JsonParseError);
  EXPECT_THROW(Json::parse("0x10"), JsonParseError);
  EXPECT_THROW(Json::parse("NaN"), JsonParseError);
  EXPECT_THROW(Json::parse("1e999"), JsonParseError);  // overflows double
}

TEST(Json, EnforcesNestingLimit)
{
  // A hostile payload must produce an error, not a stack overflow.
  const std::string deep(100000, '[');
  EXPECT_THROW(Json::parse(deep), JsonParseError);

  EXPECT_NO_THROW(Json::parse("[[[1]]]", 3));
  EXPECT_THROW(Json::parse("[[[[1]]]]", 3), JsonParseError);
}

TEST(Json, ReportsErrorOffset)
{
  try {
    Json::parse("[1, ?]");
    EXPECT_TRUE(false) << "expected JsonParseError";
  } catch (const JsonParseError & error) {
    EXPECT_EQ(error.offset(), 4u);
  }
}

TEST(Json, DumpsCompactDeterministicText)
{
  Json value;
  value["zeta"] = 1;
  value["alpha"] = "x";
  value["list"].push_back(true);
  value["list"].push_back(nullptr);
  value["list"].push_back(2.5);
  // Keys come out sorted regardless of insertion order.
  EXPECT_EQ(value.dump(), R"({"alpha":"x","list":[true,null,2.5],"zeta":1})");
}

TEST(Json, DumpsIntegersWithoutFractionAndEscapesStrings)
{
  EXPECT_EQ(Json(4294967295u).dump(), "4294967295");
  EXPECT_EQ(Json(-3).dump(), "-3");
  EXPECT_EQ(Json(0.1).dump(), "0.10000000000000001");
  EXPECT_EQ(Json("a\"b\\\n\x01").dump(), R"("a\"b\\\n\u0001")");
  EXPECT_EQ(Json(Json::Array{}).dump(), "[]");
  EXPECT_EQ(Json(Json::Object{}).dump(), "{}");
}

TEST(Json, RoundTripsThroughDumpAndParse)
{
  const std::string text =
    R"({"a":[1,2.5,-7,"s",true,false,null],"b":{"c":"\u00e9\n"},"d":1e-7})";
  const Json first = Json::parse(text);
  const Json second = Json::parse(first.dump());
  EXPECT_EQ(first, second);
}

TEST(Json, TypeMismatchThrowsLogicError)
{
  const Json number = Json::parse("1");
  EXPECT_THROW(number.as_string(), JsonTypeError);
  EXPECT_THROW(number.as_array(), JsonTypeError);
  EXPECT_EQ(number.find("key"), nullptr);  // find() on a non-object is safe
}
