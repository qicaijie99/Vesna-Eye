#pragma once
// Minimal dependency-free unit-test framework (no gtest required).
//
// Usage:
//   GT_TEST(my_test_name) {
//     GT_CHECK(1 + 1 == 2);
//     GT_CHECK_NEAR(3.0, 3.0001, 1e-3);
//   }
//
// Runs via the glowtrack_tests executable (see tests/test_main.cpp).

#include <cmath>
#include <cstdio>
#include <functional>
#include <string>
#include <vector>

namespace gttest {

struct Failure {
  std::string message;
};

struct TestCase {
  std::string name;
  std::function<void()> fn;
};

inline std::vector<TestCase>& registry() {
  static std::vector<TestCase> tests;
  return tests;
}

struct Registrar {
  Registrar(std::string name, std::function<void()> fn) {
    registry().push_back({std::move(name), std::move(fn)});
  }
};

inline int runAll() {
  int failed = 0;
  const auto& tests = registry();
  std::printf("== running %zu unit test(s) ==\n", tests.size());
  for (const auto& test : tests) {
    try {
      test.fn();
      std::printf("[ PASS ] %s\n", test.name.c_str());
    } catch (const Failure& failure) {
      ++failed;
      std::printf("[ FAIL ] %s\n         %s\n", test.name.c_str(),
                  failure.message.c_str());
    } catch (const std::exception& e) {
      ++failed;
      std::printf("[ FAIL ] %s\n         exception: %s\n", test.name.c_str(),
                  e.what());
    } catch (...) {
      ++failed;
      std::printf("[ FAIL ] %s\n         unknown exception\n",
                  test.name.c_str());
    }
  }
  std::printf("== %d/%zu passed ==\n", static_cast<int>(tests.size()) - failed,
              tests.size());
  return failed == 0 ? 0 : 1;
}

}  // namespace gttest

#define GT_TEST(name)                                              \
  static void gt_test_##name();                                    \
  static ::gttest::Registrar gt_registrar_##name(#name,            \
                                                 gt_test_##name);  \
  static void gt_test_##name()

#define GT_CHECK(cond)                                                    \
  do {                                                                    \
    if (!(cond)) {                                                        \
      throw ::gttest::Failure{std::string("CHECK failed: ") + #cond +    \
                              " (" + __FILE__ + ":" +                     \
                              std::to_string(__LINE__) + ")"};            \
    }                                                                     \
  } while (0)

#define GT_CHECK_NEAR(a, b, tol)                                          \
  do {                                                                    \
    const double gt_va = static_cast<double>(a);                          \
    const double gt_vb = static_cast<double>(b);                          \
    if (!(std::fabs(gt_va - gt_vb) <= static_cast<double>(tol))) {        \
      throw ::gttest::Failure{                                            \
          std::string("CHECK_NEAR failed: ") + #a + "=" + std::to_string(gt_va) +      \
          " vs " + #b + "=" + std::to_string(gt_vb) + " (" + __FILE__ +  \
          ":" + std::to_string(__LINE__) + ")"};                          \
    }                                                                     \
  } while (0)

#define GT_CHECK_LT(a, b)                                                 \
  do {                                                                    \
    const double gt_va = static_cast<double>(a);                          \
    const double gt_vb = static_cast<double>(b);                          \
    if (!(gt_va < gt_vb)) {                                               \
      throw ::gttest::Failure{                                            \
          std::string("CHECK_LT failed: ") + #a + "=" + std::to_string(gt_va) +        \
          " !< " + #b + "=" + std::to_string(gt_vb) + " (" + __FILE__ +  \
          ":" + std::to_string(__LINE__) + ")"};                          \
    }                                                                     \
  } while (0)

#define GT_CHECK_GT(a, b)                                                 \
  do {                                                                    \
    const double gt_va = static_cast<double>(a);                          \
    const double gt_vb = static_cast<double>(b);                          \
    if (!(gt_va > gt_vb)) {                                               \
      throw ::gttest::Failure{                                            \
          std::string("CHECK_GT failed: ") + #a + "=" + std::to_string(gt_va) +        \
          " !> " + #b + "=" + std::to_string(gt_vb) + " (" + __FILE__ +  \
          ":" + std::to_string(__LINE__) + ")"};                          \
    }                                                                     \
  } while (0)

#define GT_CHECK_LE(a, b)                                                 \
  do {                                                                    \
    const double gt_va = static_cast<double>(a);                          \
    const double gt_vb = static_cast<double>(b);                          \
    if (!(gt_va <= gt_vb)) {                                              \
      throw ::gttest::Failure{                                            \
          std::string("CHECK_LE failed: ") + #a + "=" + std::to_string(gt_va) +        \
          " !<= " + #b + "=" + std::to_string(gt_vb) + " (" + __FILE__ + \
          ":" + std::to_string(__LINE__) + ")"};                          \
    }                                                                     \
  } while (0)
