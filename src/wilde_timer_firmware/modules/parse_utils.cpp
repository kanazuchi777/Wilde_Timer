#include "parse_utils.h"

#include <cctype>
#include <cstdlib>
#include <cstring>

char *trimInPlace(char *s) {
  while (*s != '\0' && std::isspace(static_cast<unsigned char>(*s))) {
    s++;
  }
  char *end = s + strlen(s);
  while (end > s && std::isspace(static_cast<unsigned char>(*(end - 1)))) {
    end--;
  }
  *end = '\0';
  return s;
}

bool equalsIgnoreCase(const char *a, const char *b) {
  if (!a || !b) {
    return false;
  }
  while (*a != '\0' && *b != '\0') {
    const char ca = static_cast<char>(std::tolower(static_cast<unsigned char>(*a)));
    const char cb = static_cast<char>(std::tolower(static_cast<unsigned char>(*b)));
    if (ca != cb) {
      return false;
    }
    a++;
    b++;
  }
  return *a == '\0' && *b == '\0';
}

bool parseSignedLongStrict(const char *text, long &out) {
  if (!text) {
    return false;
  }
  char *end = nullptr;
  const long value = strtol(text, &end, 10);
  if (end == text) {
    return false;
  }
  while (*end != '\0' && std::isspace(static_cast<unsigned char>(*end))) {
    end++;
  }
  if (*end != '\0') {
    return false;
  }
  out = value;
  return true;
}

bool parseUnsignedLongStrict(const char *text, unsigned long &out) {
  if (!text) {
    return false;
  }
  char *end = nullptr;
  const unsigned long value = strtoul(text, &end, 10);
  if (end == text) {
    return false;
  }
  while (*end != '\0' && std::isspace(static_cast<unsigned char>(*end))) {
    end++;
  }
  if (*end != '\0') {
    return false;
  }
  out = value;
  return true;
}

bool parseBoolStrict(const char *text, bool &out) {
  if (!text) {
    return false;
  }
  if (equalsIgnoreCase(text, "1") || equalsIgnoreCase(text, "true") || equalsIgnoreCase(text, "yes") ||
      equalsIgnoreCase(text, "on")) {
    out = true;
    return true;
  }
  if (equalsIgnoreCase(text, "0") || equalsIgnoreCase(text, "false") || equalsIgnoreCase(text, "no") ||
      equalsIgnoreCase(text, "off")) {
    out = false;
    return true;
  }
  return false;
}

bool parseUsRangeStrict(const char *text, uint16_t &minUs, uint16_t &maxUs) {
  if (!text) {
    return false;
  }
  char buf[40] = {};
  snprintf(buf, sizeof(buf), "%s", text);
  char *dash = strchr(buf, '-');
  if (!dash) {
    return false;
  }
  *dash = '\0';
  char *left = trimInPlace(buf);
  char *right = trimInPlace(dash + 1);
  unsigned long lo = 0;
  unsigned long hi = 0;
  if (!parseUnsignedLongStrict(left, lo) || !parseUnsignedLongStrict(right, hi)) {
    return false;
  }
  if (lo > 65535UL || hi > 65535UL) {
    return false;
  }
  minUs = static_cast<uint16_t>(lo);
  maxUs = static_cast<uint16_t>(hi);
  return true;
}

bool parseOsdPointStrict(const char *text, uint8_t &col, uint8_t &row) {
  if (!text) {
    return false;
  }
  char buf[40] = {};
  snprintf(buf, sizeof(buf), "%s", text);
  char *s = trimInPlace(buf);
  const size_t len = strlen(s);
  if (len < 5 || s[0] != '[' || s[len - 1] != ']') {
    return false;
  }
  s[len - 1] = '\0';
  char *comma = strchr(s + 1, ',');
  if (!comma) {
    return false;
  }
  *comma = '\0';
  char *left = trimInPlace(s + 1);
  char *right = trimInPlace(comma + 1);
  unsigned long colVal = 0;
  unsigned long rowVal = 0;
  if (!parseUnsignedLongStrict(left, colVal) || !parseUnsignedLongStrict(right, rowVal)) {
    return false;
  }
  if (colVal > 255UL || rowVal > 255UL) {
    return false;
  }
  col = static_cast<uint8_t>(colVal);
  row = static_cast<uint8_t>(rowVal);
  return true;
}

bool parseOsdPointWithFlagStrict(const char *text, uint8_t &col, uint8_t &row, bool &showDuringRace, bool &hasFlag) {
  if (!text) {
    return false;
  }
  char buf[64] = {};
  snprintf(buf, sizeof(buf), "%s", text);
  char *s = trimInPlace(buf);
  const size_t len = strlen(s);
  if (len < 5 || s[0] != '[' || s[len - 1] != ']') {
    return false;
  }

  s[len - 1] = '\0';
  char *firstComma = strchr(s + 1, ',');
  if (!firstComma) {
    return false;
  }
  *firstComma = '\0';
  char *left = trimInPlace(s + 1);

  char *rest = trimInPlace(firstComma + 1);
  char *secondComma = strchr(rest, ',');

  char *mid = rest;
  char *right = nullptr;
  hasFlag = false;
  showDuringRace = true;

  if (secondComma) {
    *secondComma = '\0';
    right = trimInPlace(secondComma + 1);
    if (strchr(right, ',') != nullptr) {
      return false;
    }
    hasFlag = true;
  }
  mid = trimInPlace(mid);

  unsigned long colVal = 0;
  unsigned long rowVal = 0;
  if (!parseUnsignedLongStrict(left, colVal) || !parseUnsignedLongStrict(mid, rowVal)) {
    return false;
  }
  if (colVal > 255UL || rowVal > 255UL) {
    return false;
  }

  if (hasFlag) {
    bool flagVal = true;
    if (!parseBoolStrict(right, flagVal)) {
      return false;
    }
    showDuringRace = flagVal;
  }

  col = static_cast<uint8_t>(colVal);
  row = static_cast<uint8_t>(rowVal);
  return true;
}
