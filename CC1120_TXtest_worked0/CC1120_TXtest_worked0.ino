#include <SPI.h>


#define R_BIT           0x80
#define EXT_ADDR        0x2F
#define SS_PIN          4
#define MARCSTATE       0x73 //MARCSTATE(通信機の状態)

uint8_t w;

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

SPISettings settings(1000, MSBFIRST, SPI_MODE0);

uint8_t readExtAddrSPI(uint8_t addr);

void setup() {
  pinMode(SS_PIN, OUTPUT);
  digitalWrite(SS_PIN, HIGH);
  // put your setup code here, to run once:
  Serial.begin(9600);
  SPI.begin();
  // SPI.beginTransaction(SPISettings(500000, MSBFIRST, SPI_MODE0));

  strobeSPI(0x30);  //Reset chip

  
  strobeSPI(0x36); //Exit RX/TX, turn off frequency synthesizer and exit eWOR mode if applicable

  configureCC1120();
  w = readExtAddrSPI(MARCSTATE);
  
  Serial.print("MARCSTATE:");
  Serial.println(w, BIN);

}

void loop() {
  configureCC1120();
  digitalWrite(SS_PIN, HIGH);
  // put your main code here, to run repeatedly:
  w = readExtAddrSPI(0x8F);
  Serial.print("PARTNUMBER:");
  Serial.println(w, HEX);
  Serial.print("PARTVERSION:");
  w = readExtAddrSPI(0x90);
  Serial.println(w, HEX);
  Serial.print("MARCSTATE:");
  w = readExtAddrSPI(MARCSTATE);
  Serial.println(w, BIN);
  Serial.print("FREQ:");
  w = readExtAddrSPI(0x0C);
  Serial.print(w, HEX);
  w = readExtAddrSPI(0x0D);
  Serial.print(w, HEX);
  w = readExtAddrSPI(0x0E);
  Serial.println(w, HEX);
  w = readSPI(0x21);
  Serial.print("FS_CFG:");
  Serial.println(w, HEX);
  w = readSPI(0x2B);
  Serial.print("PA_CFG2:");
  Serial.println(w, HEX);
  for(int i=0; i<128; i++){
    for(uint32_t i=0; i<128; i++){
      writeSPI(0x3F, 1110011);
    }
    strobeSPI(0x35);  // Enable TX
    delay(10);
    Serial.print("MARCSTATE:");
    w = readExtAddrSPI(MARCSTATE);
    Serial.println(w, BIN);
    strobeSPI(0x36);
    FIFOFlush();
  }
  // strobeSPI(0x35);

  w = readExtAddrSPI(MARCSTATE);
  Serial.print("MARCSTATE:");
  Serial.println(w, BIN);
  
  // delay(1000);  

  // strobeSPI(0x36);

  // FIFOFlush();

  // delay(1000);
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

  

  uint32_t FREQ = (uint32_t) (437*pow(2, 11)*8);

  Serial.println(FREQ, HEX);
  // Serial.println(FREQ >> 8, HEX);
  // Serial.println(FREQ >> 16, HEX);
  delay(100);
 
  //writeExtAddrSPI(0x0E, FREQ); // FREQ0の設定
  //writeExtAddrSPI(0x0D, FREQ >> 8); // FREQ1の設定
  //writeExtAddrSPI(0x0C, FREQ >> 16); // FREQ2の設定
  writeExtAddrSPI(0x0C, FREQ >> 16); // FREQ2の設定
  writeExtAddrSPI(0x0D, FREQ >> 8); // FREQ1の設定
  writeExtAddrSPI(0x0E, FREQ); // FREQ0の設定
 

  writeSPI(0x2B, 0x3F); // PA_CFG2レジスタを設定  // 出力パワーを設定するレジスタです。
  writeSPI(0x21, 0x14); //Frequency Synthesizer Configuration
  writeSPI(0x0B, 0x23); //Modulation Format and Frequency Deviation Configuration
 
  // 必要な他のレジスタを適切に設定する

  strobeSPI(0x33); //Calibrate frequency synthesizer and turn it off

  delay(100);
}

void FIFOFlush(){
  strobeSPI(0x3A); // Flush the RX FIFO
  strobeSPI(0x3B); // Flush the TX FIFO
}
