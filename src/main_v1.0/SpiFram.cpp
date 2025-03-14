#include "SpiFram.h"
#include "SPI.h"

// TODO メモリ容量的にdefineにしたい
const byte CMD_WREN = 0x06;  // 0000 0110 Set Write Enable Latch
const byte CMD_WRDI = 0x04;  // 0000 0100 Write Disable
const byte CMD_RDSR = 0x05;  // 0000 0101 Read Status Register
const byte CMD_WRSR = 0x01;  // 0000 0001 Write Status Register
const byte CMD_READ = 0x03;  // 0000 0011 Read Memory Data
const byte CMD_WRITE = 0x02; // 0000 0010 Write Memory Data

SpiFram::SpiFram()
{
}

// アドレスをどこで渡すべきか
void SpiFram::begin()
{
    // TODO SPI5クロックの確認　main.inoで宣言してるクロックで動作するか？
    SPI5.begin();
    SPI5.setDataMode(SPI_MODE0);
    SPI5.setBitOrder(MSBFIRST);
    SPI5.setClockDivider(SPI_CLOCK_DIV2);
}

void SpiFram::reset(byte reset_data)
{
    decoder.write(2);         // /CSをLowに
    SPI5.transfer(CMD_WREN);  // 書き込みイネーブルコマンド
    decoder.write(7);         // /CSをHighに
    decoder.write(2);         // /CSをLowに
    SPI5.transfer(CMD_WRITE); // write command
    SPI5.transfer(0);
    SPI5.transfer(0);
    SPI5.transfer(0);
    for (unsigned int i = 0; i < pow(2, 16) - 1; i++)
    {
        SPI5.transfer(reset_data);
    }
    decoder.write(7); // /CSをHighに
}

void SpiFram::write(unsigned long eeaddress, byte data)
{
    decoder.write(2);         // /CSをLowに
    SPI5.transfer(CMD_WREN);  // 書き込みイネーブルコマンド
    decoder.write(7);         // /CSをHighに
    decoder.write(2);         // /CSをLowに
    SPI5.transfer(CMD_WRITE); // write command
    SPI5.transfer((int)(eeaddress >> 16));
    SPI5.transfer((int)(eeaddress >> 8) & 0xFF);
    SPI5.transfer((int)((eeaddress) & 0xFF) & 0xFF);
    SPI5.transfer(data);
    decoder.write(7); // /CSをHighに
}

byte SpiFram::read(unsigned long eeaddress)
{
    byte rdata = 0x00;
    decoder.write(2); // /CSをLowに
    SPI5.transfer(CMD_READ);
    SPI5.transfer((int)(eeaddress >> 16));
    SPI5.transfer((int)(eeaddress >> 8) & 0xFF);
    SPI5.transfer((int)((eeaddress) & 0xFF) & 0xFF);
    rdata = SPI5.transfer(0x00);
    decoder.write(7); // /CSをHighに
    return rdata;
}
