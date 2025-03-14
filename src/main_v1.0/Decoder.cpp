#include "Arduino.h"
#include "Decoder.h"

Decoder::Decoder()
{
}

void Decoder::init()
{
    pinMode(PIN_A, OUTPUT);
    pinMode(PIN_B, OUTPUT);
    pinMode(PIN_C, OUTPUT);
    //NOTE デコーダを使用しない時はY7をLOWにする
    write(7);
}

void Decoder::write(int pin)
{
    digitalWrite(PIN_A, pin & 0x01); // LSB
    digitalWrite(PIN_B, (pin >> 1) & 0x01);
    digitalWrite(PIN_C, (pin >> 2) & 0x01); // MSB
}