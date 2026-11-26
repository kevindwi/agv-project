#pragma once

#include <Arduino.h>

class SerialParser {
public:
  SerialParser();

  /**
   * Mengumpulkan byte dari serial.
   * Mengembalikan true jika satu baris perintah 'C,linear,angular' lengkap dan valid telah terbaca.
   */
  bool feed(char c, float &linear, float &angular);

  void reset();

private:
  bool parseLine(char *line, float &linear, float &angular);

  char _buffer[48];
  byte _index;
  bool _discardLine;
};
