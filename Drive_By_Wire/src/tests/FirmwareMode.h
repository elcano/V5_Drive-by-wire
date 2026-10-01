#pragma once

#ifndef DBW_SERIAL_TEST_MODE
#define DBW_SERIAL_TEST_MODE 0
#endif

#if DBW_SERIAL_TEST_MODE != 0 && DBW_SERIAL_TEST_MODE != 1
#error "DBW_SERIAL_TEST_MODE must be 0 (normal) or 1 (serial tests)"
#endif