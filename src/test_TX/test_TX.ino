#include <SPI.h>
#include "CC1120_addr.h"
 
#define R_BIT           0x80
#define SS_PIN          20
#define LoRa_PIN        21
#define DECA_PIN        19
#define DECB_PIN        25
#define DECC_PIN        22
 
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
  pinMode(LoRa_PIN, OUTPUT);
  pinMode(DECA_PIN, OUTPUT);
  pinMode(DECB_PIN, OUTPUT);
  pinMode(DECC_PIN, OUTPUT);
  digitalWrite(SS_PIN, HIGH);
  digitalWrite(LoRa_PIN, HIGH);
  digitalWrite(DECA_PIN, HIGH);
  digitalWrite(DECB_PIN, HIGH);
  digitalWrite(DECC_PIN, HIGH);
 
  Serial.begin(9600);
  SPI5.begin();
 
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
    for(uint32_t i=0; i<64; i++){
      writeSPI(TXRX_FIFO, 0b11101011);
    }
    for(uint32_t i=0; i<64; i++){
      writeSPI(TXRX_FIFO, 0b00000110);
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
  ccstatus.v = SPI5.transfer(R_BIT | addr);
  uint8_t v = SPI5.transfer(0x00);
  digitalWrite(SS_PIN, HIGH);
  return v;
}
 
void writeSPI(uint8_t addr, uint8_t value) {
  digitalWrite(SS_PIN, LOW);
  ccstatus.v = SPI5.transfer(addr);
  ccstatus.v = SPI5.transfer(value);
  digitalWrite(SS_PIN, HIGH);
}
 
void strobeSPI(uint8_t cmd)
{
  digitalWrite(SS_PIN, LOW);
  ccstatus.v = SPI5.transfer(R_BIT | cmd);
  digitalWrite(SS_PIN, HIGH);
}
 
uint8_t readExtAddrSPI(uint8_t addr) {
  static uint8_t v;
  SPI5.beginTransaction(settings);
  digitalWrite(SS_PIN, LOW);
  ccstatus.v = SPI5.transfer(R_BIT | EXT_ADDR);
  SPI5.transfer(addr);
  delayMicroseconds(10);
  v = SPI5.transfer(0xff);
  // Serial.println(SPI5.transfer(0xff), BIN);
  digitalWrite(SS_PIN, HIGH);
  SPI5.endTransaction();
  return v;
}
 
void writeExtAddrSPI(uint8_t addr, uint8_t value) {
  digitalWrite(SS_PIN, LOW);
  ccstatus.v = SPI5.transfer(EXT_ADDR);
  ccstatus.v = SPI5.transfer(addr);
  ccstatus.v = SPI5.transfer(value);
  digitalWrite(SS_PIN, HIGH);
}
 
/*void configureCC1120() {
 
  writeSPI(IOCFG3, 0xB0);
  writeSPI(IOCFG2, 0x06);
  writeSPI(IOCFG1, 0xB0);
  writeSPI(IOCFG0, 0x40);
 
  writeSPI(SYNC_CFG1, 0x0B);
  writeSPI(DEVIATION_M, 0x0D);
  writeSPI(MODCFG_DEV_E, 0x00);
 
  writeSPI(DCFILT_CFG, 0x1C);
  writeSPI(PREAMBLE_CFG1, 0x18);
  writeSPI(IQIC, 0xC6);
  writeSPI(CHAN_BW, 0x08);
  writeSPI(MDMCFG0, 0x05);
  writeSPI(SYMBOL_RATE2, 0x62);
  writeSPI(SYMBOL_RATE1, 0x6E);
  writeSPI(SYMBOL_RATE0, 0x98);
 
  writeSPI(AGC_CFG1, 0xA9);
  writeSPI(AGC_CFG0, 0xCF);
  writeSPI(FIFO_CFG, 0x00);
  writeSPI(FS_CFG, 0x14);
  writeSPI(PKT_CFG0, 0x20);
  writeSPI(PA_CFG2, 0x3F);
  writeSPI(PA_CFG0, 0x7E);
  writeSPI(IF_MIX_CFG, 0x00);
 
  writeExtAddrSPI(FREQOFF_CFG, 0x22);
  uint32_t FREQ = (uint32_t)(0x6B8000);
 
  Serial.print("FREQ in configration: ");
  Serial.println(FREQ, HEX);
  delay(100);
 
  writeExtAddrSPI(FREQ2, FREQ >> 16); // FREQ2の設定
  writeExtAddrSPI(FREQ1, FREQ >> 8); // FREQ1の設定
  writeExtAddrSPI(FREQ0, FREQ); // FREQ0の設定
 
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
}*/

void configureCC1120_2nd(){
writeSPI(IOCFG3, 0xB0);        // GPIO3 IO Pin Configuration
writeSPI(IOCFG2, 0x06);        // GPIO2 IO Pin Configuration
writeSPI(IOCFG1, 0xB0);        // GPIO1 IO Pin Configuration
writeSPI(IOCFG0, 0x40);        // GPIO0 IO Pin Configuration
writeSPI(SYNC_CFG1, 0x0B);     // Sync Word Detection Configuration Reg. 1
writeSPI(DEVIATION_M, 0x89);   // Frequency Deviation Configuration
writeSPI(MODCFG_DEV_E, 0x01);   // Frequency Deviation Configuration
writeSPI(DCFILT_CFG, 0x1C);    // Digital DC Removal Configuration
writeSPI(PREAMBLE_CFG1, 0x18); // Preamble Length Configuration Reg. 1
writeSPI(IQIC, 0xC6);          // Digital Image Channel Compensation Configuration
writeSPI(CHAN_BW, 0x08);       // Channel Filter Configuration
writeSPI(MDMCFG0, 0x05);       // General Modem Parameter Configuration Reg. 0
writeSPI(SYMBOL_RATE2, 0x48);  // Symbol Rate Configuration Exponent and Mantissa [1..
writeSPI(SYMBOL_RATE1, 0x93);  // Symbol Rate Configuration Mantissa [15:8]
writeSPI(SYMBOL_RATE0, 0x75);  // Symbol Rate Configuration Mantissa [7:0]
writeSPI(AGC_REF, 0x20);       // AGC Reference Level Configuration
writeSPI(AGC_CS_THR, 0x19);    // Carrier Sense Threshold Configuration
writeSPI(AGC_CFG1, 0xA9);      // Automatic Gain Control Configuration Reg. 1
writeSPI(AGC_CFG0, 0xCF);      // Automatic Gain Control Configuration Reg. 0
writeSPI(FIFO_CFG, 0x00);      // FIFO Configuration
writeSPI(FS_CFG, 0x14);        // Frequency Synthesizer Configuration
writeSPI(PKT_CFG0, 0x20);      // Packet Configuration Reg. 0
writeSPI(PA_CFG0, 0x7E);       // Power Amplifier Configuration Reg. 0
writeSPI(PKT_LEN, 0xFF);       // Packet Length Configuration
writeSPI(IF_MIX_CFG, 0x00);    // IF Mix Configuration

writeExtAddrSPI(FREQOFF_CFG, 0x22);   // Frequency Offset Correction Configuration
writeExtAddrSPI(FREQ2, 0x6D);         // Frequency Configuration [23:16]
writeExtAddrSPI(FREQ1, 0x43);         // Frequency Configuration [15:8]
writeExtAddrSPI(FREQ0, 0x33);         // Frequency Configuration [7:0]
writeExtAddrSPI(FS_DIG1, 0x00);       // Frequency Synthesizer Digital Reg. 1
writeExtAddrSPI(FS_DIG0, 0x5F);       // Frequency Synthesizer Digital Reg. 0
writeExtAddrSPI(FS_CAL1, 0x40);       // Frequency Synthesizer Calibration Reg. 1
writeExtAddrSPI(FS_CAL0, 0x0E);       // Frequency Synthesizer Calibration Reg. 0
writeExtAddrSPI(FS_DIVTWO, 0x03);     // Frequency Synthesizer Divide by 2
writeExtAddrSPI(FS_DSM0, 0x33);       // FS Digital Synthesizer Module Configuration Reg. 0
writeExtAddrSPI(FS_DVC0, 0x17);       // Frequency Synthesizer Divider Chain Configuration ..
writeExtAddrSPI(FS_PFD, 0x50);        // Frequency Synthesizer Phase Frequency Detector Con..
writeExtAddrSPI(FS_PRE, 0x6E);        // Frequency Synthesizer Prescaler Configuration
writeExtAddrSPI(FS_REG_DIV_CML, 0x14);// Frequency Synthesizer Divider Regulator Configurat..
writeExtAddrSPI(FS_SPARE, 0xAC);      // Frequency Synthesizer Spare
writeExtAddrSPI(FS_VCO0, 0xB4);       // FS Voltage Controlled Oscillator Configuration Reg..
writeExtAddrSPI(XOSC5, 0x0E);         // Crystal Oscillator Configuration Reg. 5
writeExtAddrSPI(XOSC1, 0x03);         // Crystal Oscillator Configuration Reg. 1
writeExtAddrSPI(PARTNUMBER, 0x48);    // Part Number
writeExtAddrSPI(PARTVERSION, 0x23);   // Part Revision
writeExtAddrSPI(MODEM_STATUS1, 0x10); // Modem Status Reg. 1




  strobeSPI(SCAL); //Calibrate frequency synthesizer and turn it off
 
  delay(100);
}
 
void FIFOFlush(){
  strobeSPI(SFRX); // Flush the RX FIFO
  strobeSPI(SFTX); // Flush the TX FIFO
}
 