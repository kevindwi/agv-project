#include "serial_parser.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

SerialParser::SerialParser() : _index(0), _discardLine(false) {
  _buffer[0] = '\0';
}

void SerialParser::reset() {
  _index = 0;
  _discardLine = false;
  _buffer[0] = '\0';
}

bool SerialParser::feed(char c, float &linear, float &angular) {
  if (c == '\n') {
    bool valid = false;
    if (!_discardLine && _index > 0) {
      _buffer[_index] = '\0';
      valid = parseLine(_buffer, linear, angular);
    }
    _index = 0;
    _discardLine = false;
    return valid;
  }

  if (c != '\r') {
    if (_index < sizeof(_buffer) - 1) {
      _buffer[_index++] = c;
    } else {
      _discardLine = true;
    }
  }

  return false;
}

bool SerialParser::parseLine(char *line, float &linear, float &angular) {
  if (line[0] != 'C' || line[1] != ',') {
    return false;
  }

  char *linearText = line + 2;
  char *comma = strchr(linearText, ',');
  if (comma == nullptr) {
    return false;
  }

  *comma = '\0';
  char *endLinear;
  char *endAngular;

  // AVR libc menyediakan strtod; strtof tidak tersedia pada toolchain ini.
  linear = (float)strtod(linearText, &endLinear);
  angular = (float)strtod(comma + 1, &endAngular);

  if (endLinear == linearText || endAngular == comma + 1 ||
      *endLinear != '\0' || *endAngular != '\0' ||
      !isfinite(linear) || !isfinite(angular)) {
    return false;
  }

  return true;
}
