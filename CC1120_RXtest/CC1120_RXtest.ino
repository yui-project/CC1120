#include <SPI.h>
#include "CC1120_addr.h"
 
#define R_BIT           0x80
#define SS_PIN          4
 
uint8_t w;
uint32_t freq;
 
struct cc_status {
 uint8_t res : 4;
 uint8_t state : 3;
 uint8_t chip_ready : 1;
};
union cc_st {
  struct cc_status ccst;
  uint8_t v;
};
union cc_st ccstatus;
 
SPISettings settings(1000000, MSBFIRST, SPI_MODE0);
uint8_t readExtAddrSPI(uint8_t addr);
// uint8_t readMARCSTATE();
 
void setup() {
  pinMode(SS_PIN, OUTPUT);
  digitalWrite(SS_PIN, HIGH);
 
  Serial.begin(9600);
  SPI.begin();
 
  strobeSPI(SRES); //Reset chip
  strobeSPI(SIDLE); //Exit RX/TX, turn off frequency synthesizer and exit eWOR mode if applicable
 
  Serial.println();
  Serial.print("MARCSTATE before first cfg: ");
  // Serial.println(readExtAddrSPI(MARCSTATE), BIN);
  readMARCSTATE();
  configureCC1120_2nd();
  Serial.print("MARCSTATE after  first cfg: ");
  readMARCSTATE();
}
 
void loop() {
  Serial.print("MARCSTATE before cfg:  ");
  readMARCSTATE();
 
  configureCC1120_2nd();
  digitalWrite(SS_PIN, HIGH);
 
  Serial.print("MARCSTATE after  cfg:  ");
  readMARCSTATE();
 
  Serial.print("PARTNUMBER: ");
  Serial.println(readExtAddrSPI(PARTNUMBER), HEX);
 
  Serial.print("PARTVERSION: ");
  Serial.println(readExtAddrSPI(PARTVERSION), HEX);
 
  Serial.print("FREQ: ");
  freq = readExtAddrSPI(FREQ2) << 16 | readExtAddrSPI(FREQ1) << 8 | readExtAddrSPI(FREQ0);
  Serial.println(freq, HEX);
 
  Serial.print("FS_CFG: ");
  Serial.println(readSPI(FS_CFG), HEX);
 
  Serial.print("PA_CFG2: ");
  Serial.println(readSPI(PA_CFG2), HEX);
 
  for(int i=0; i<128; i++){
    Serial.print("MARCSTATE before SRX:  ");
    readMARCSTATE();
   
    strobeSPI(SRX);  // Enable RX
    delay(100);

    uint8_t RXByte = 0;
    while(RXByte < 15){
      RXByte = readExtAddrSPI(FIFO_NUM_RXBYTES);
      delay(1000);
      Serial.print("RXByte: ");
      Serial.println(RXByte);
    }

    uint8_t RXData = 1;
    for(uint32_t i=0; i<128; i++){
      RXByte = readExtAddrSPI(FIFO_NUM_RXBYTES);
      if(RXByte < 1){
        break;
      }
      RXByte = readExtAddrSPI(FIFO_NUM_RXBYTES);
      Serial.print("RXByte: ");
      Serial.println(RXByte);
      RXData = readSPI(0b10111111); // R/~W[7], Burst[6], RX_FIFO_Address[5:0]
      Serial.print("RXData 0b: ");
      Serial.println(RXData, HEX);
      RXData = readSPI(0x3F); // R/~W[7], Burst[6], RX_FIFO_Address[5:0]
      Serial.print("RXData 3F: ");
      Serial.println(RXData, HEX);
      
      Serial.print("MARCSTATE after ReadRX: ");
      readMARCSTATE();
      delay(1000);
    }
   
    Serial.print("MARCSTATE after  SRX:  ");
    readMARCSTATE();
   
    strobeSPI(SIDLE); // Exit TX/RX, turn off frequency synthesizer and exit eWOR mode if applicable
    FIFOFlush();
 
    delay(10);
    Serial.print("MARCSTATE after SIDLE: ");
    readMARCSTATE();
  }
}
 
void readMARCSTATE(){
  uint8_t value = readExtAddrSPI(MARCSTATE);
  uint8_t mask = 0b11111;
  uint8_t state = value & mask;
 
  if(state == 0b00001){
    Serial.print(value, BIN);
    Serial.println(" , IDLE");
  }
  else if(state == 0b01101){
    Serial.print(value, BIN);
    Serial.println(" , RX");
  }
  else if(state == 0b01110){
    Serial.print(value, BIN);
    Serial.println(" , RX_END");
  }
  else if(state == 0b10011){
    Serial.print("0");
    Serial.print(value, BIN);
    Serial.println(" , TX");
  }
  else if(state == 0b10100){
    Serial.print("0");
    Serial.print(value, BIN);
    Serial.println(" , TX_END");
  }
  else{
    Serial.println(value, BIN);
  }
}
 
uint8_t readSPI(uint8_t addr) {
  digitalWrite(SS_PIN, LOW);
  ccstatus.v = SPI.transfer(R_BIT | addr);
  uint8_t v = SPI.transfer(0x00);
  digitalWrite(SS_PIN, HIGH);
  return v;
}
 
void writeSPI(uint8_t addr, uint8_t value) {
  digitalWrite(SS_PIN, LOW);
  ccstatus.v = SPI.transfer(addr);
  ccstatus.v = SPI.transfer(value);
  digitalWrite(SS_PIN, HIGH);
}
 
void strobeSPI(uint8_t cmd)
{
  digitalWrite(SS_PIN, LOW);
  ccstatus.v = SPI.transfer(R_BIT | cmd);
  digitalWrite(SS_PIN, HIGH);
}
 
uint8_t readExtAddrSPI(uint8_t addr) {
  static uint8_t v;
  SPI.beginTransaction(settings);
  digitalWrite(SS_PIN, LOW);
  ccstatus.v = SPI.transfer(R_BIT | EXT_ADDR);
  SPI.transfer(addr);
  delayMicroseconds(10);
  v = SPI.transfer(0xff);
  // Serial.println(SPI.transfer(0xff), BIN);
  digitalWrite(SS_PIN, HIGH);
  SPI.endTransaction();
  return v;
}
 
void writeExtAddrSPI(uint8_t addr, uint8_t value) {
  digitalWrite(SS_PIN, LOW);
  ccstatus.v = SPI.transfer(EXT_ADDR);
  ccstatus.v = SPI.transfer(addr);
  ccstatus.v = SPI.transfer(value);
  digitalWrite(SS_PIN, HIGH);
}

void configureCC1120_2nd(){
writeSPI(IOCFG3, 0xB0);
writeSPI(IOCFG2, 0x06);
writeSPI(IOCFG1, 0xB0);
writeSPI(IOCFG0, 0x40);
writeSPI(SYNC_CFG1, 0x0B);
writeSPI(DCFILT_CFG, 0x1C);
writeSPI(PREAMBLE_CFG1, 0x18);
writeSPI(IQIC, 0xC6);
writeSPI(CHAN_BW, 0x08);
writeSPI(MDMCFG0, 0x05);
writeSPI(AGC_REF, 0x20);
writeSPI(AGC_CS_THR, 0x19);
writeSPI(AGC_CFG1, 0xA9);
writeSPI(AGC_CFG0, 0xCF);
writeSPI(FIFO_CFG, 0x00);
writeSPI(FS_CFG, 0x14);
writeSPI(PKT_CFG0, 0x20);
writeSPI(PKT_LEN, 0xFF);

writeExtAddrSPI(IF_MIX_CFG, 0x00);
writeExtAddrSPI(FREQOFF_CFG, 0x22);
writeExtAddrSPI(FREQ2, 0x6B);
writeExtAddrSPI(FREQ1, 0x80);
writeExtAddrSPI(FS_DIG1, 0x00);
writeExtAddrSPI(FS_DIG0, 0x5F);
writeExtAddrSPI(FS_CAL1, 0x40);
writeExtAddrSPI(FS_CAL0, 0x0E);
writeExtAddrSPI(FS_DIVTWO, 0x03);
writeExtAddrSPI(FS_DSM0, 0x33);
writeExtAddrSPI(FS_DVC0, 0x17);
writeExtAddrSPI(FS_PFD, 0x50);
writeExtAddrSPI(FS_PRE, 0x6E);
writeExtAddrSPI(FS_REG_DIV_CML, 0x14);
writeExtAddrSPI(FS_SPARE, 0xAC);
writeExtAddrSPI(FS_VCO0, 0xB4);
writeExtAddrSPI(XOSC5, 0x0E);
writeExtAddrSPI(XOSC1, 0x03);

strobeSPI(SCAL); //Calibrate frequency synthesizer and turn it off

delay(100);
}
 
void FIFOFlush(){
  strobeSPI(SFRX); // Flush the RX FIFO
  strobeSPI(SFTX); // Flush the TX FIFO
}
 
