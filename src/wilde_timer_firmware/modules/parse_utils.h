#pragma once

#include <Arduino.h>

char *trimInPlace(char *s);
bool equalsIgnoreCase(const char *a, const char *b);
bool parseSignedLongStrict(const char *text, long &out);
bool parseUnsignedLongStrict(const char *text, unsigned long &out);
bool parseBoolStrict(const char *text, bool &out);
bool parseUsRangeStrict(const char *text, uint16_t &minUs, uint16_t &maxUs);
bool parseOsdPointStrict(const char *text, uint8_t &col, uint8_t &row);
