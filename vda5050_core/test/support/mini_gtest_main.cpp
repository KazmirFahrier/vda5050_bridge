// Copyright 2026 vda5050_bridge contributors
// SPDX-License-Identifier: Apache-2.0
//
// main() for the mini_gtest fallback. With real GoogleTest, gtest_main is linked instead.

#include <gtest/gtest.h>

int main()
{
  return ::testing::internal::run_all_tests();
}
