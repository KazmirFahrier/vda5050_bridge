// Copyright 2026 vda5050_bridge contributors
// SPDX-License-Identifier: Apache-2.0
//
// mini_gtest: a tiny stand-in for <gtest/gtest.h>.
//
// It is used ONLY when real GoogleTest is not installed (CMake prints a status
// line when that happens). Under colcon/ament and in CI the tests compile
// against real GoogleTest; this header is not on the include path there.
//
// Supported subset: TEST, TEST_F, ::testing::Test with SetUp/TearDown,
// EXPECT_/ASSERT_ {TRUE, FALSE, EQ, NE, LT, LE, GT, GE, NEAR, DOUBLE_EQ,
// THROW, NO_THROW} and streaming extra context with operator<<.
// Unlike GoogleTest it does not print the compared values on failure.

#ifndef MINI_GTEST__GTEST_H_
#define MINI_GTEST__GTEST_H_

#include <cmath>
#include <functional>
#include <iostream>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace testing
{

class Test
{
public:
  virtual ~Test() = default;
  void Run()
  {
    SetUp();
    TestBody();
    TearDown();
  }

protected:
  virtual void SetUp() {}
  virtual void TearDown() {}
  virtual void TestBody() = 0;
};

namespace internal
{

struct TestCase
{
  std::string name;
  std::function<void()> run;
};

inline std::vector<TestCase> & registry()
{
  static std::vector<TestCase> tests;
  return tests;
}

inline bool & current_test_failed()
{
  static bool failed = false;
  return failed;
}

struct Registrar
{
  Registrar(std::string name, std::function<void()> run)
  {
    registry().push_back(TestCase{std::move(name), std::move(run)});
  }
};

class Message
{
public:
  template<typename T>
  Message & operator<<(const T & value)
  {
    stream_ << value;
    return *this;
  }
  std::string str() const {return stream_.str();}

private:
  std::ostringstream stream_;
};

// Assigning a Message to an AssertHelper reports the failure. Using operator=
// (lower precedence than <<) lets `EXPECT_X(...) << "context"` work.
class AssertHelper
{
public:
  AssertHelper(const char * file, int line, const char * expression)
  : file_(file), line_(line), expression_(expression) {}

  void operator=(const Message & message) const
  {
    current_test_failed() = true;
    std::cerr << file_ << ":" << line_ << ": Failure\n  " << expression_;
    const std::string extra = message.str();
    if (!extra.empty()) {
      std::cerr << "\n  " << extra;
    }
    std::cerr << std::endl;
  }

private:
  const char * file_;
  int line_;
  const char * expression_;
};

template<typename Exception, typename Callable>
bool throws(Callable && callable)
{
  try {
    callable();
  } catch (const Exception &) {
    return true;
  } catch (...) {
    return false;
  }
  return false;
}

template<typename Callable>
bool does_not_throw(Callable && callable)
{
  try {
    callable();
  } catch (...) {
    return false;
  }
  return true;
}

inline bool double_eq(double a, double b)
{
  return a == b || std::fabs(a - b) <= 4 * 2.220446049250313e-16 * std::fmax(std::fabs(a), std::fabs(b));
}

inline int run_all_tests()
{
  int failed = 0;
  for (const TestCase & test : registry()) {
    current_test_failed() = false;
    std::cout << "[ RUN      ] " << test.name << std::endl;
    try {
      test.run();
    } catch (const std::exception & error) {
      current_test_failed() = true;
      std::cerr << "  unexpected exception: " << error.what() << std::endl;
    } catch (...) {
      current_test_failed() = true;
      std::cerr << "  unexpected non-standard exception" << std::endl;
    }
    if (current_test_failed()) {
      ++failed;
      std::cout << "[  FAILED  ] " << test.name << std::endl;
    } else {
      std::cout << "[       OK ] " << test.name << std::endl;
    }
  }
  std::cout << "[==========] " << registry().size() << " tests ran, " << failed << " failed."
            << std::endl;
  return failed == 0 ? 0 : 1;
}

}  // namespace internal
}  // namespace testing

#define MINI_GTEST_CLASS_(suite, name) suite ## _ ## name ## _Test

#define MINI_GTEST_DEFINE_(suite, name, base) \
  class MINI_GTEST_CLASS_(suite, name) : public base \
  { \
    void TestBody() override; \
  }; \
  static const ::testing::internal::Registrar mini_gtest_registrar_ ## suite ## _ ## name( \
    #suite "." #name, [] {MINI_GTEST_CLASS_(suite, name) test; test.Run();}); \
  void MINI_GTEST_CLASS_(suite, name)::TestBody()

#define TEST(suite, name) MINI_GTEST_DEFINE_(suite, name, ::testing::Test)
#define TEST_F(fixture, name) MINI_GTEST_DEFINE_(fixture, name, fixture)

// The `switch (0) case 0: default:` prefix makes the macro a single statement
// that is safe inside an un-braced if/else (the same trick GoogleTest uses).
#define MINI_GTEST_EXPECT_(condition, text) \
  switch (0) case 0: default: \
    if (condition) {} else \
      ::testing::internal::AssertHelper(__FILE__, __LINE__, text) = ::testing::internal::Message()

#define MINI_GTEST_ASSERT_(condition, text) \
  switch (0) case 0: default: \
    if (condition) {} else \
      return ::testing::internal::AssertHelper(__FILE__, __LINE__, text) = \
             ::testing::internal::Message()

#define EXPECT_TRUE(c) MINI_GTEST_EXPECT_(static_cast<bool>(c), "Expected true: " #c)
#define EXPECT_FALSE(c) MINI_GTEST_EXPECT_(!static_cast<bool>(c), "Expected false: " #c)
#define EXPECT_EQ(a, b) MINI_GTEST_EXPECT_((a) == (b), "Expected equal: " #a " and " #b)
#define EXPECT_NE(a, b) MINI_GTEST_EXPECT_((a) != (b), "Expected not equal: " #a " and " #b)
#define EXPECT_LT(a, b) MINI_GTEST_EXPECT_((a) < (b), "Expected: " #a " < " #b)
#define EXPECT_LE(a, b) MINI_GTEST_EXPECT_((a) <= (b), "Expected: " #a " <= " #b)
#define EXPECT_GT(a, b) MINI_GTEST_EXPECT_((a) > (b), "Expected: " #a " > " #b)
#define EXPECT_GE(a, b) MINI_GTEST_EXPECT_((a) >= (b), "Expected: " #a " >= " #b)
#define EXPECT_NEAR(a, b, tolerance) \
  MINI_GTEST_EXPECT_(std::fabs((a) - (b)) <= (tolerance), "Expected near: " #a " and " #b)
#define EXPECT_DOUBLE_EQ(a, b) \
  MINI_GTEST_EXPECT_(::testing::internal::double_eq((a), (b)), "Expected equal: " #a " and " #b)
#define EXPECT_THROW(statement, exception) \
  MINI_GTEST_EXPECT_( \
    ::testing::internal::throws<exception>([&] {statement;}), \
    "Expected " #statement " to throw " #exception)
#define EXPECT_NO_THROW(statement) \
  MINI_GTEST_EXPECT_( \
    ::testing::internal::does_not_throw([&] {statement;}), \
    "Expected " #statement " not to throw")

#define ASSERT_TRUE(c) MINI_GTEST_ASSERT_(static_cast<bool>(c), "Expected true: " #c)
#define ASSERT_FALSE(c) MINI_GTEST_ASSERT_(!static_cast<bool>(c), "Expected false: " #c)
#define ASSERT_EQ(a, b) MINI_GTEST_ASSERT_((a) == (b), "Expected equal: " #a " and " #b)
#define ASSERT_NE(a, b) MINI_GTEST_ASSERT_((a) != (b), "Expected not equal: " #a " and " #b)
#define ASSERT_LT(a, b) MINI_GTEST_ASSERT_((a) < (b), "Expected: " #a " < " #b)
#define ASSERT_LE(a, b) MINI_GTEST_ASSERT_((a) <= (b), "Expected: " #a " <= " #b)
#define ASSERT_GT(a, b) MINI_GTEST_ASSERT_((a) > (b), "Expected: " #a " > " #b)
#define ASSERT_GE(a, b) MINI_GTEST_ASSERT_((a) >= (b), "Expected: " #a " >= " #b)

#endif  // MINI_GTEST__GTEST_H_
