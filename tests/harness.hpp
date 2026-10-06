// Minimal self-contained test harness (no external dependency).
#pragma once

#include <cmath>
#include <cstdio>
#include <vector>

namespace fmtest {

struct Case {
    const char* name;
    void (*fn)();
};

inline std::vector<Case>& registry() {
    static std::vector<Case> cases;
    return cases;
}

inline int& failures() {
    static int count = 0;
    return count;
}

struct Registrar {
    Registrar(const char* name, void (*fn)()) { registry().push_back({name, fn}); }
};

inline void check(bool ok, const char* expr, const char* file, int line) {
    if (ok) return;
    ++failures();
    std::printf("    FAILED %s:%d: %s\n", file, line, expr);
}

inline void check_near(double a, double b, double tol, const char* expr, const char* file,
                       int line) {
    if (std::abs(a - b) <= tol) return;
    ++failures();
    std::printf("    FAILED %s:%d: %s (%g vs %g, tolerance %g)\n", file, line, expr, a, b, tol);
}

}  // namespace fmtest

#define FMTEST_CONCAT2(a, b) a##b
#define FMTEST_CONCAT(a, b) FMTEST_CONCAT2(a, b)
#define TEST_CASE(name)                                                                     \
    static void FMTEST_CONCAT(fmtest_fn_, __LINE__)();                                      \
    static const ::fmtest::Registrar FMTEST_CONCAT(fmtest_reg_, __LINE__)(                  \
        name, &FMTEST_CONCAT(fmtest_fn_, __LINE__));                                        \
    static void FMTEST_CONCAT(fmtest_fn_, __LINE__)()
#define CHECK(expr) ::fmtest::check(static_cast<bool>(expr), #expr, __FILE__, __LINE__)
#define CHECK_NEAR(a, b, tol) \
    ::fmtest::check_near((a), (b), (tol), #a " ~= " #b, __FILE__, __LINE__)
