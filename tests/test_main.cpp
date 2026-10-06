#include <cstring>

#include "harness.hpp"

// Usage: fm_tests [substring]  -- runs the cases whose name contains the substring.
int main(int argc, char** argv) {
    const char* filter = argc > 1 ? argv[1] : nullptr;
    int run = 0;
    int failed_cases = 0;
    for (const auto& c : fmtest::registry()) {
        if (filter && !std::strstr(c.name, filter)) continue;
        const int before = fmtest::failures();
        std::printf("[ RUN  ] %s\n", c.name);
        c.fn();
        const bool ok = fmtest::failures() == before;
        std::printf("[ %s ] %s\n", ok ? " OK " : "FAIL", c.name);
        ++run;
        if (!ok) ++failed_cases;
    }
    std::printf("\n%d test case(s), %d failed\n", run, failed_cases);
    return failed_cases == 0 && run > 0 ? 0 : 1;
}
