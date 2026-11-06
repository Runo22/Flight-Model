// Stand-in (tests/resim only): the log line on stdout.
#pragma once
#include <cstdio>
#include <format>

#define RESIM_LOG(category, level, ...) std::puts(std::format(__VA_ARGS__).c_str())
