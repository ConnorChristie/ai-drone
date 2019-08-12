#pragma once

#include "ocv_common.hpp"

#ifdef _WIN32
#define CALL(windows_fn, unix_fn, args) windows_fn(args)
#else
#define CALL(windows_fn, unix_fn, args) unix_fn(args)
#endif

#if defined(_WIN32)
bool launchDebugger();
#endif
