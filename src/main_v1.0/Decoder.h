#pragma once
#include "Arduino.h"

//NOTE:メインボードのデコーダ用のクラス　IoExpander、MCP3204のクラスで使用するためデコーダ用のクラスを作成した

class Decoder {
private:
#define PIN_A 19
#define PIN_B 25
#define PIN_C 22
public:
  Decoder();
  void init();
  void write(int pin);
};