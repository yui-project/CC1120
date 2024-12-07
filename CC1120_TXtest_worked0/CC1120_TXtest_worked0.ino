#include <SPI.h>
#include <CC1120_addr.h>

#define R_BIT           0x80
#define SS_PIN          4

uint8_t w ,w1, w2, w3;
float freq;

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

void setup() {
  pinMode(SS_PIN, OUTPUT);
  digitalWrite(SS_PIN, HIGH);
 
  Serial.begin(9600);
  SPI.begin();

  strobeSPI(SRES); //Reset chip
  strobeSPI(SIDLE); //Exit RX/TX, turn off frequency synthesizer and exit eWOR mode if applicable
  
  Serial.print("MARCSTATE before first cfg: ");
  Serial.print(readExtAddrSPI(MARCSTATE), BIN);
  configureCC1120();
  Serial.print("MARCSTATE after  first cfg: ");
  Serial.println(readExtAddrSPI(MARCSTATE), BIN);
}

void loop() {
  configureCC1120();
  digitalWrite(SS_PIN, HIGH);
  // put your main code here, to run repeatedly:
  w = readExtAddrSPI(PARTNUMBER);
  Serial.print("PARTNUMBER: ");
  Serial.println(w, HEX);
 
  Serial.print("PARTVERSION: ");
  w = readExtAddrSPI(PARTVERSION);
  Serial.println(w, HEX);

  Serial.print("MARCSTATE: ");
  w = readExtAddrSPI(MARCSTATE);
  Serial.println(w, BIN);
 
  Serial.print("FREQ: ");
  w1 = readExtAddrSPI(FREQ2);
  w2 = readExtAddrSPI(FREQ1);
  w3 = readExtAddrSPI(FREQ0);
  freq = ((uint32_t)w1 << 16) | ((uint32_t)w2 << 8) | (uint32_t)w3;
  Serial.println(freq, HEX);
  
  Serial.print("FS_CFG: ");
  w = readSPI(FS_CFG);
  Serial.println(w, HEX);
  
  Serial.print("PA_CFG2: ");
  w = readSPI(PA_CFG2);
  Serial.println(w, HEX);
  
  for(int i=0; i<128; i++){
    for(uint32_t i=0; i<128; i++){
      writeSPI(TXRX_FIFO, 1110011);
    }
    Serial.print("MARCSTATE before STX: ");
    w = readExtAddrSPI(MARCSTATE);
    Serial.println(w, BIN);
    
    strobeSPI(STX);  // Enable TX
    delay(10);
    
    Serial.print("MARCSTATE after STX: ");
    w = readExtAddrSPI(MARCSTATE);
    Serial.println(w, BIN);
    
    strobeSPI(SIDLE); // Exit TX/RX, turn off frequency synthesizer and exit eWOR mode if applicable
    FIFOFlush();
  }

  w = readExtAddrSPI(MARCSTATE);
  Serial.print("MARCSTATE:");
  Serial.println(w, BIN);
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
  Serial.print("FREQ in configration:");
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
