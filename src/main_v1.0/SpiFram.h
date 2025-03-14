#pragma once
#include "Arduino.h"
#include "Decoder.h"


class SpiFram
{
private:
    Decoder decoder;

public:
    SpiFram();
    void begin();
    void reset(byte reset_data);
    void write(unsigned long eeaddress, byte data);
    byte read(unsigned long eeaddress);
};