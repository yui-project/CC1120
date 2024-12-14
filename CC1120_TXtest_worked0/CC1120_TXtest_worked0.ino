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
  configureCC1120();
  Serial.print("MARCSTATE after  first cfg: ");
  readMARCSTATE();
}

void loop() {
  Serial.print("MARCSTATE before cfg:  ");
  readMARCSTATE();

  configureCC1120();
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
    for(uint32_t i=0; i<128; i++){
      writeSPI(TXRX_FIFO, 0b1110011);
    }
    Serial.print("MARCSTATE before STX:  ");
    readMARCSTATE();
    
    strobeSPI(STX);  // Enable TX
    delay(10);
    
    Serial.print("MARCSTATE after  STX:  ");
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

void configureCC1120() {
  // 430MHz帯での設定を行う
  uint32_t FREQ = (uint32_t)(0x6B8000);
  Serial.print("FREQ in configration: ");
  Serial.println(FREQ, HEX);
  delay(100);
 
  writeExtAddrSPI(FREQ2, FREQ >> 16); // FREQ2の設定
  writeExtAddrSPI(FREQ1, FREQ >> 8); // FREQ1の設定
  writeExtAddrSPI(FREQ0, FREQ); // FREQ0の設定
 
  // 必要な他のレジスタを適切に設定する
  writeSPI(PA_CFG2, 0x3F); // PA_CFG2レジスタを設定  // 出力パワーを設定するレジスタです。
  writeSPI(FS_CFG, 0x14); //Frequency Synthesizer Configuration
  writeSPI(MODCFG_DEV_E, 0x23); //Modulation Format and Frequency Deviation Configuration

  strobeSPI(SCAL); //Calibrate frequency synthesizer and turn it off

  delay(100);
}

void FIFOFlush(){
  strobeSPI(SFRX); // Flush the RX FIFO
  strobeSPI(SFTX); // Flush the TX FIFO
}
