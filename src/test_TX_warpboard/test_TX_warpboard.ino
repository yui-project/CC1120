#include <SPI.h>
#include "CC1120_addr.h"
 
#define R_BIT           0x80
#define SS_PIN          4
#define LoRa_PIN        21
#define DECA_PIN        19
#define DECB_PIN        25
#define DECC_PIN        22
 
uint8_t w;
uint32_t freq;
 
struct cc_status {
 uint8_t res : 4;
 uint8_t state : 3;
//  uint8_t chip_ready : 1;
};
union cc_st {
  struct cc_status ccst;                                                                                                                                                                                                                     
  uint8_t v;
};
union cc_st ccstatus;
 
SPISettings settings(100000, MSBFIRST, SPI_MODE0);
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
  SPI.begin();
 
  
}
 
void loop() {

  resetCC1120();
  delay(10);
  strobeSPI(SIDLE); //Exit RX/TX, turn off frequency synthesizer and exit eWOR mode if applicable
  delay(10);
 
  Serial.println();
  Serial.print("MARCSTATE before first cfg: ");
  // Serial.println(readExtAddrSPI(MARCSTATE), BIN);
  readMARCSTATE();
  configureCC1120_2nd();
  Serial.print("MARCSTATE after  first cfg: ");
  readMARCSTATE();

  Serial.print("MARCSTATE before cfg:  ");
  readMARCSTATE();

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
 
  char buffer[128] = "aaariwarano narihira. ariwarano narihira. ariwarano narihira. ariwarano narihira. ariwarano narihira. ariwarano narihira. ariwarano narihira. ariwarano narihira.";
  int index = 2;
  // while (Serial.available() == 0){
  //   delay(10);
  // }
  // while (Serial.available()>0) {
  //   buffer[index] = Serial.read();
  //   index++;
  //   delay(10);
  //   //バッファ以上の場合は中断
  //   // if (index >= 128) {
  //   //   break;
  //   // }
  // }
  // // index--;
  int randLen = random(126);
  randLen = 19;
  for(int i=0; i<randLen; i++){
    // buffer[i] = (char)random(128);
    index++;
  }

  buffer[0] = index-1;
  buffer[1] = 0x55;
  buffer[2] = 0x02;
  buffer[20] = 0x04;

  for(int i=0; i<1; i++){
    FIFOFlush();
    

    for(uint32_t i=0; i<index; i++){
      writeSPI(TXRX_FIFO, buffer[i]);
      Serial.print(buffer[i]);
    }
    Serial.println();


    Serial.println(index);
    Serial.println(readExtAddrSPI(TXFIRST));
    Serial.println(readExtAddrSPI(TXLAST));
    writeExtAddrSPI(TXLAST, index);
    Serial.println(readExtAddrSPI(TXLAST));
    Serial.println(readSPI(PKT_LEN));
    writeSPI(PKT_LEN, index);
    Serial.println(readSPI(PKT_LEN));

    // for(int i=0; i<128; i++){
    //   Serial.print(readDirectFIFO(i));
    // }
    Serial.print("MARCSTATE before STX:  ");
    readMARCSTATE();
   
    strobeSPI(STX);  // Enable TX
    delay(2000);
   
    Serial.print("MARCSTATE after  STX:  ");
    readMARCSTATE();

    Serial.println(readExtAddrSPI(TXFIRST), HEX);
    Serial.println(readExtAddrSPI(TXLAST), HEX);
   
    strobeSPI(SIDLE); // Exit TX/RX, turn off frequency synthesizer and exit eWOR mode if applicable
    
 
    delay(10000);
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
  SPI.beginTransaction(settings);
  digitalWrite(SS_PIN, LOW);
  ccstatus.v = SPI.transfer(R_BIT | addr);
  uint8_t v = SPI.transfer(0x00);
  digitalWrite(SS_PIN, HIGH);
  SPI.endTransaction();
  return v;
}
 
void writeSPI(uint8_t addr, uint8_t value) {
  SPI.beginTransaction(settings);
  digitalWrite(SS_PIN, LOW);
  ccstatus.v = SPI.transfer(addr);
  ccstatus.v = SPI.transfer(value);
  digitalWrite(SS_PIN, HIGH);
  SPI.endTransaction();
}
 
void strobeSPI(uint8_t cmd)
{
  SPI.beginTransaction(settings);
  digitalWrite(SS_PIN, LOW);
  ccstatus.v = SPI.transfer(R_BIT | cmd);
  digitalWrite(SS_PIN, HIGH);
  SPI.endTransaction();
}

void resetCC1120(){
  SPI.beginTransaction(settings);
  digitalWrite(SS_PIN, LOW);
  ccstatus.v = SPI.transfer(SRES);
  delay(1);
  while(digitalRead(17) == 1){
    delay(1);
  }
  digitalWrite(SS_PIN, HIGH);
  SPI.endTransaction();
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
  SPI.beginTransaction(settings);
  digitalWrite(SS_PIN, LOW);
  ccstatus.v = SPI.transfer(EXT_ADDR);
  ccstatus.v = SPI.transfer(addr);
  ccstatus.v = SPI.transfer(value);
  digitalWrite(SS_PIN, HIGH);
  SPI.endTransaction();
}

uint8_t readDirectFIFO(uint8_t addr) {
  static uint8_t v;
  SPI.beginTransaction(settings);
  digitalWrite(SS_PIN, LOW);
  ccstatus.v = SPI.transfer(DIRECT_MEMORY_ACCESS);
  SPI.transfer(addr);
  delayMicroseconds(10);
  v = SPI.transfer(0xff);
  // Serial.println(SPI.transfer(0xff), BIN);
  digitalWrite(SS_PIN, HIGH);
  SPI.endTransaction();
  return v;
}
 
void writeDirectFIFO(uint8_t addr, uint8_t value) {
  SPI.beginTransaction(settings);
  digitalWrite(SS_PIN, LOW);
  ccstatus.v = SPI.transfer(DIRECT_MEMORY_ACCESS);
  ccstatus.v = SPI.transfer(addr);
  ccstatus.v = SPI.transfer(value);
  digitalWrite(SS_PIN, HIGH);
  SPI.endTransaction();
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
writeSPI(IOCFG3, IOCFG3_VALUE);
writeSPI(IOCFG2, IOCFG2_VALUE);
writeSPI(IOCFG1, IOCFG1_VALUE);
writeSPI(IOCFG0, IOCFG0_VALUE);
writeSPI(SYNC3, SYNC3_VALUE);
writeSPI(SYNC2, SYNC2_VALUE);
writeSPI(SYNC1, SYNC1_VALUE);
writeSPI(SYNC0, SYNC0_VALUE);
writeSPI(SYNC_CFG1, SYNC_CFG1_VALUE);
writeSPI(SYNC_CFG0, SYNC_CFG0_VALUE);
writeSPI(DEVIATION_M, DEVIATION_M_VALUE);
writeSPI(MODCFG_DEV_E, MODCFG_DEV_E_VALUE);
writeSPI(DCFILT_CFG, DCFILT_CFG_VALUE);
writeSPI(PREAMBLE_CFG1, PREAMBLE_CFG1_VALUE);
writeSPI(PREAMBLE_CFG0, PREAMBLE_CFG0_VALUE);
writeSPI(FREQ_IF_CFG, FREQ_IF_CFG_VALUE);
writeSPI(IQIC, IQIC_VALUE);
writeSPI(CHAN_BW, CHAN_BW_VALUE);
writeSPI(MDMCFG1, MDMCFG1_VALUE);
writeSPI(MDMCFG0, MDMCFG0_VALUE);
writeSPI(SYMBOL_RATE2, SYMBOL_RATE2_VALUE);
writeSPI(SYMBOL_RATE1, SYMBOL_RATE1_VALUE);
writeSPI(SYMBOL_RATE0, SYMBOL_RATE0_VALUE);
writeSPI(AGC_REF, AGC_REF_VALUE);
writeSPI(AGC_CS_THR, AGC_CS_THR_VALUE);
writeSPI(AGC_GAIN_ADJUST, AGC_GAIN_ADJUST_VALUE);
writeSPI(AGC_CFG3, AGC_CFG3_VALUE);
writeSPI(AGC_CFG2, AGC_CFG2_VALUE);
writeSPI(AGC_CFG1, AGC_CFG1_VALUE);
writeSPI(AGC_CFG0, AGC_CFG0_VALUE);
writeSPI(FIFO_CFG, FIFO_CFG_VALUE);
writeSPI(DEV_ADDR, DEV_ADDR_VALUE);
writeSPI(SETTLING_CFG, SETTLING_CFG_VALUE);
writeSPI(FS_CFG, FS_CFG_VALUE);
writeSPI(WOR_CFG1, WOR_CFG1_VALUE);
writeSPI(WOR_CFG0, WOR_CFG0_VALUE);
writeSPI(WOR_EVENT0_MSB, WOR_EVENT0_MSB_VALUE);
writeSPI(WOR_EVENT0_LSB, WOR_EVENT0_LSB_VALUE);
writeSPI(PKT_CFG2, PKT_CFG2_VALUE);
writeSPI(PKT_CFG1, PKT_CFG1_VALUE);
writeSPI(PKT_CFG0, PKT_CFG0_VALUE);
writeSPI(RFEND_CFG1, RFEND_CFG1_VALUE);
writeSPI(RFEND_CFG0, RFEND_CFG0_VALUE);
writeSPI(PA_CFG2, PA_CFG2_VALUE);
writeSPI(PA_CFG1, PA_CFG1_VALUE);
writeSPI(PA_CFG0, PA_CFG0_VALUE);
writeSPI(PKT_LEN, PKT_LEN_VALUE);

writeExtAddrSPI(FREQOFF_CFG, FREQOFF_CFG_VALUE);
writeExtAddrSPI(TOC_CFG, TOC_CFG_VALUE);
writeExtAddrSPI(MARC_SPARE, MARC_SPARE_VALUE);
writeExtAddrSPI(ECG_CFG, ECG_CFG_VALUE);
writeExtAddrSPI(CFM_DATA_CFG, CFM_DATA_CFG_VALUE);
writeExtAddrSPI(EXT_CTRL, EXT_CTRL_VALUE);
writeExtAddrSPI(RCCAL_FINE, RCCAL_FINE_VALUE);
writeExtAddrSPI(RCCAL_COARSE, RCCAL_COARSE_VALUE);
writeExtAddrSPI(RCCAL_OFFSET, RCCAL_OFFSET_VALUE);
writeExtAddrSPI(FREQOFF1, FREQOFF1_VALUE);
writeExtAddrSPI(FREQOFF0, FREQOFF0_VALUE);
writeExtAddrSPI(FREQ2, FREQ2_VALUE);
writeExtAddrSPI(FREQ1, FREQ1_VALUE);
writeExtAddrSPI(FREQ0, FREQ0_VALUE);
writeExtAddrSPI(IF_ADC2, IF_ADC2_VALUE);
writeExtAddrSPI(IF_ADC1, IF_ADC1_VALUE);
writeExtAddrSPI(IF_ADC0, IF_ADC0_VALUE);
writeExtAddrSPI(FS_DIG1, FS_DIG1_VALUE);
writeExtAddrSPI(FS_DIG0, FS_DIG0_VALUE);
writeExtAddrSPI(FS_CAL3, FS_CAL3_VALUE);
writeExtAddrSPI(FS_CAL2, FS_CAL2_VALUE);
writeExtAddrSPI(FS_CAL1, FS_CAL1_VALUE);
writeExtAddrSPI(FS_CAL0, FS_CAL0_VALUE);
writeExtAddrSPI(FS_CHP, FS_CHP_VALUE);
writeExtAddrSPI(FS_DIVTWO, FS_DIVTWO_VALUE);
writeExtAddrSPI(FS_DSM1, FS_DSM1_VALUE);
writeExtAddrSPI(FS_DSM0, FS_DSM0_VALUE);
writeExtAddrSPI(FS_DVC1, FS_DVC1_VALUE);
writeExtAddrSPI(FS_DVC0, FS_DVC0_VALUE);
writeExtAddrSPI(FS_LBI, FS_LBI_VALUE);
writeExtAddrSPI(FS_PFD, FS_PFD_VALUE);
writeExtAddrSPI(FS_PRE, FS_PRE_VALUE);
writeExtAddrSPI(FS_REG_DIV_CML, FS_REG_DIV_CML_VALUE);
writeExtAddrSPI(FS_SPARE, FS_SPARE_VALUE);
writeExtAddrSPI(FS_VCO4, FS_VCO4_VALUE);
writeExtAddrSPI(FS_VCO3, FS_VCO3_VALUE);
writeExtAddrSPI(FS_VCO2, FS_VCO2_VALUE);
writeExtAddrSPI(FS_VCO1, FS_VCO1_VALUE);
writeExtAddrSPI(FS_VCO0, FS_VCO0_VALUE);
writeExtAddrSPI(GBIAS6, GBIAS6_VALUE);
writeExtAddrSPI(GBIAS5, GBIAS5_VALUE);
writeExtAddrSPI(GBIAS4, GBIAS4_VALUE);
writeExtAddrSPI(GBIAS3, GBIAS3_VALUE);
writeExtAddrSPI(GBIAS2, GBIAS2_VALUE);
writeExtAddrSPI(GBIAS1, GBIAS1_VALUE);
writeExtAddrSPI(GBIAS0, GBIAS0_VALUE);
writeExtAddrSPI(IFAMP, IFAMP_VALUE);
writeExtAddrSPI(LNA, LNA_VALUE);
writeExtAddrSPI(RXMIX, RXMIX_VALUE);
writeExtAddrSPI(XOSC5, XOSC5_VALUE);
writeExtAddrSPI(XOSC4, XOSC4_VALUE);
writeExtAddrSPI(XOSC3, XOSC3_VALUE);
writeExtAddrSPI(XOSC2, XOSC2_VALUE);
writeExtAddrSPI(XOSC1, XOSC1_VALUE);
writeExtAddrSPI(XOSC0, XOSC0_VALUE);
writeExtAddrSPI(ANALOG_SPARE, ANALOG_SPARE_VALUE);
writeExtAddrSPI(PA_CFG3, PA_CFG3_VALUE);
writeExtAddrSPI(WOR_TIME1, WOR_TIME1_VALUE);
writeExtAddrSPI(WOR_TIME0, WOR_TIME0_VALUE);
writeExtAddrSPI(WOR_CAPTURE1, WOR_CAPTURE1_VALUE);
writeExtAddrSPI(WOR_CAPTURE0, WOR_CAPTURE0_VALUE);
writeExtAddrSPI(BIST, BIST_VALUE);
writeExtAddrSPI(DCFILTOFFSET_I1, DCFILTOFFSET_I1_VALUE);
writeExtAddrSPI(DCFILTOFFSET_I0, DCFILTOFFSET_I0_VALUE);
writeExtAddrSPI(DCFILTOFFSET_Q1, DCFILTOFFSET_Q1_VALUE);
writeExtAddrSPI(DCFILTOFFSET_Q0, DCFILTOFFSET_Q0_VALUE);
writeExtAddrSPI(IQIE_I1, IQIE_I1_VALUE);
writeExtAddrSPI(IQIE_I0, IQIE_I0_VALUE);
writeExtAddrSPI(IQIE_Q1, IQIE_Q1_VALUE);
writeExtAddrSPI(IQIE_Q0, IQIE_Q0_VALUE);
writeExtAddrSPI(RSSI1, RSSI1_VALUE);
writeExtAddrSPI(RSSI0, RSSI0_VALUE);
writeExtAddrSPI(MARCSTATE, MARCSTATE_VALUE);
writeExtAddrSPI(LQI_VAL, LQI_VAL_VALUE);
writeExtAddrSPI(PQT_SYNC_ERR, PQT_SYNC_ERR_VALUE);
writeExtAddrSPI(DEM_STATUS, DEM_STATUS_VALUE);
writeExtAddrSPI(FREQOFF_EST1, FREQOFF_EST1_VALUE);
writeExtAddrSPI(FREQOFF_EST0, FREQOFF_EST0_VALUE);
writeExtAddrSPI(AGC_GAIN3, AGC_GAIN3_VALUE);
writeExtAddrSPI(AGC_GAIN2, AGC_GAIN2_VALUE);
writeExtAddrSPI(AGC_GAIN1, AGC_GAIN1_VALUE);
writeExtAddrSPI(AGC_GAIN0, AGC_GAIN0_VALUE);
writeExtAddrSPI(CFM_RX_DATA_OUT, CFM_RX_DATA_OUT_VALUE);
writeExtAddrSPI(CFM_TX_DATA_IN, CFM_TX_DATA_IN_VALUE);
writeExtAddrSPI(ASK_SOFT_RX_DATA, ASK_SOFT_RX_DATA_VALUE);
writeExtAddrSPI(RNDGEN, RNDGEN_VALUE);
writeExtAddrSPI(MAGN2, MAGN2_VALUE);
writeExtAddrSPI(MAGN1, MAGN1_VALUE);
writeExtAddrSPI(MAGN0, MAGN0_VALUE);
writeExtAddrSPI(ANG1, ANG1_VALUE);
writeExtAddrSPI(ANG0, ANG0_VALUE);
writeExtAddrSPI(CHFILT_I2, CHFILT_I2_VALUE);
writeExtAddrSPI(CHFILT_I1, CHFILT_I1_VALUE);
writeExtAddrSPI(CHFILT_I0, CHFILT_I0_VALUE);
writeExtAddrSPI(CHFILT_Q2, CHFILT_Q2_VALUE);
writeExtAddrSPI(CHFILT_Q1, CHFILT_Q1_VALUE);
writeExtAddrSPI(CHFILT_Q0, CHFILT_Q0_VALUE);
writeExtAddrSPI(GPIO_STATUS, GPIO_STATUS_VALUE);
writeExtAddrSPI(FSCAL_CTRL, FSCAL_CTRL_VALUE);
writeExtAddrSPI(PHASE_ADJUST, PHASE_ADJUST_VALUE);
// writeExtAddrSPI(PARTNUMBER, PARTNUMBER_VALUE);
// writeExtAddrSPI(PARTVERSION, PARTVERSION_VALUE);
writeExtAddrSPI(SERIAL_STATUS, SERIAL_STATUS_VALUE);
writeExtAddrSPI(MODEM_STATUS1, MODEM_STATUS1_VALUE);
writeExtAddrSPI(MODEM_STATUS0, MODEM_STATUS0_VALUE);
writeExtAddrSPI(MARC_STATUS1, MARC_STATUS1_VALUE);
writeExtAddrSPI(MARC_STATUS0, MARC_STATUS0_VALUE);
writeExtAddrSPI(PA_IFAMP_TEST, PA_IFAMP_TEST_VALUE);
writeExtAddrSPI(FSRF_TEST, FSRF_TEST_VALUE);
writeExtAddrSPI(PRE_TEST, PRE_TEST_VALUE);
writeExtAddrSPI(PRE_OVR, PRE_OVR_VALUE);
writeExtAddrSPI(ADC_TEST, ADC_TEST_VALUE);
writeExtAddrSPI(DVC_TEST, DVC_TEST_VALUE);
writeExtAddrSPI(ATEST, ATEST_VALUE);
writeExtAddrSPI(ATEST_LVDS, ATEST_LVDS_VALUE);
writeExtAddrSPI(ATEST_MODE, ATEST_MODE_VALUE);
writeExtAddrSPI(XOSC_TEST1, XOSC_TEST1_VALUE);
writeExtAddrSPI(XOSC_TEST0, XOSC_TEST0_VALUE);
// writeExtAddrSPI(RXFIRST, RXFIRST_VALUE);
// writeExtAddrSPI(TXFIRST, TXFIRST_VALUE);
// writeExtAddrSPI(RXLAST, RXLAST_VALUE);
// writeExtAddrSPI(TXLAST, TXLAST_VALUE);
// writeExtAddrSPI(NUM_TXBYTES, NUM_TXBYTES_VALUE);
// writeExtAddrSPI(NUM_RXBYTES, NUM_RXBYTES_VALUE);
// writeExtAddrSPI(FIFO_NUM_TXBYTES, FIFO_NUM_TXBYTES_VALUE);
// writeExtAddrSPI(FIFO_NUM_RXBYTES, FIFO_NUM_RXBYTES_VALUE);



int ret = 1;

if(readSPI(IOCFG3)           != IOCFG3_VALUE              ) ret=0;
if(readSPI(IOCFG2)           != IOCFG2_VALUE              ) ret=0;
if(readSPI(IOCFG1)           != IOCFG1_VALUE              ) ret=0;
if(readSPI(IOCFG0)           != IOCFG0_VALUE              ) ret=0;
if(readSPI(SYNC3)            != SYNC3_VALUE               ) ret=0;
if(readSPI(SYNC2)            != SYNC2_VALUE               ) ret=0;
if(readSPI(SYNC1)            != SYNC1_VALUE               ) ret=0;
if(readSPI(SYNC0)            != SYNC0_VALUE               ) ret=0;
if(readSPI(SYNC_CFG1)        != SYNC_CFG1_VALUE           ) ret=0;
if(readSPI(SYNC_CFG0)        != SYNC_CFG0_VALUE           ) ret=0;
if(readSPI(DEVIATION_M)      != DEVIATION_M_VALUE         ) ret=0;
if(readSPI(MODCFG_DEV_E)     != MODCFG_DEV_E_VALUE        ) ret=0;
if(readSPI(DCFILT_CFG)       != DCFILT_CFG_VALUE          ) ret=0;
if(readSPI(PREAMBLE_CFG1)    != PREAMBLE_CFG1_VALUE       ) ret=0;
if(readSPI(PREAMBLE_CFG0)    != PREAMBLE_CFG0_VALUE       ) ret=0;
if(readSPI(FREQ_IF_CFG)      != FREQ_IF_CFG_VALUE         ) ret=0;
if(readSPI(IQIC)             != IQIC_VALUE                ) ret=0;
if(readSPI(CHAN_BW)          != CHAN_BW_VALUE             ) ret=0;
if(readSPI(MDMCFG1)          != MDMCFG1_VALUE             ) ret=0;
if(readSPI(MDMCFG0)          != MDMCFG0_VALUE             ) ret=0;
if(readSPI(SYMBOL_RATE2)     != SYMBOL_RATE2_VALUE        ) ret=0;
if(readSPI(SYMBOL_RATE1)     != SYMBOL_RATE1_VALUE        ) ret=0;
if(readSPI(SYMBOL_RATE0)     != SYMBOL_RATE0_VALUE        ) ret=0;
if(readSPI(AGC_REF)          != AGC_REF_VALUE             ) ret=0;
if(readSPI(AGC_CS_THR)       != AGC_CS_THR_VALUE          ) ret=0;
if(readSPI(AGC_GAIN_ADJUST)  != AGC_GAIN_ADJUST_VALUE     ) ret=0;
if(readSPI(AGC_CFG3)         != AGC_CFG3_VALUE            ) ret=0;
if(readSPI(AGC_CFG2)         != AGC_CFG2_VALUE            ) ret=0;
if(readSPI(AGC_CFG1)         != AGC_CFG1_VALUE            ) ret=0;
if(readSPI(AGC_CFG0)         != AGC_CFG0_VALUE            ) ret=0;
if(readSPI(FIFO_CFG)         != FIFO_CFG_VALUE            ) ret=0;
if(readSPI(DEV_ADDR)         != DEV_ADDR_VALUE            ) ret=0;
if(readSPI(SETTLING_CFG)     != SETTLING_CFG_VALUE        ) ret=0;
if(readSPI(FS_CFG)           != FS_CFG_VALUE              ) ret=0;
if(readSPI(WOR_CFG1)         != WOR_CFG1_VALUE            ) ret=0;
if(readSPI(WOR_CFG0)         != WOR_CFG0_VALUE            ) ret=0;
if(readSPI(WOR_EVENT0_MSB)   != WOR_EVENT0_MSB_VALUE      ) ret=0;
if(readSPI(WOR_EVENT0_LSB)   != WOR_EVENT0_LSB_VALUE      ) ret=0;
if(readSPI(PKT_CFG2)         != PKT_CFG2_VALUE            ) ret=0;
if(readSPI(PKT_CFG1)         != PKT_CFG1_VALUE            ) ret=0;
if(readSPI(PKT_CFG0)         != PKT_CFG0_VALUE            ) ret=0;
if(readSPI(RFEND_CFG1)       != RFEND_CFG1_VALUE          ) ret=0;
if(readSPI(RFEND_CFG0)       != RFEND_CFG0_VALUE          ) ret=0;
if(readSPI(PA_CFG2)          != PA_CFG2_VALUE             ) ret=0;
if(readSPI(PA_CFG1)          != PA_CFG1_VALUE             ) ret=0;
if(readSPI(PA_CFG0)          != PA_CFG0_VALUE             ) ret=0;
if(readSPI(PKT_LEN)          != PKT_LEN_VALUE             ) ret=0;



if(readExtAddrSPI(IF_MIX_CFG)       != IF_MIX_CFG_VALUE          ) ret=0;
if(readExtAddrSPI(FREQOFF_CFG)      != FREQOFF_CFG_VALUE         ) ret=0;
if(readExtAddrSPI(TOC_CFG)          != TOC_CFG_VALUE             ) ret=0;
if(readExtAddrSPI(MARC_SPARE)       != MARC_SPARE_VALUE          ) ret=0;
if(readExtAddrSPI(ECG_CFG)          != ECG_CFG_VALUE             ) ret=0;
if(readExtAddrSPI(CFM_DATA_CFG)     != CFM_DATA_CFG_VALUE        ) ret=0;
if(readExtAddrSPI(EXT_CTRL)         != EXT_CTRL_VALUE            ) ret=0;
if(readExtAddrSPI(RCCAL_FINE)       != RCCAL_FINE_VALUE          ) ret=0;
if(readExtAddrSPI(RCCAL_COARSE)     != RCCAL_COARSE_VALUE        ) ret=0;
if(readExtAddrSPI(RCCAL_OFFSET)     != RCCAL_OFFSET_VALUE        ) ret=0;
if(readExtAddrSPI(FREQOFF1)         != FREQOFF1_VALUE            ) ret=0;
if(readExtAddrSPI(FREQOFF0)         != FREQOFF0_VALUE            ) ret=0;
if(readExtAddrSPI(FREQ2)            != FREQ2_VALUE               ) ret=0;
if(readExtAddrSPI(FREQ1)            != FREQ1_VALUE               ) ret=0;
if(readExtAddrSPI(FREQ0)            != FREQ0_VALUE               ) ret=0;
if(readExtAddrSPI(IF_ADC2)          != IF_ADC2_VALUE             ) ret=0;
if(readExtAddrSPI(IF_ADC1)          != IF_ADC1_VALUE             ) ret=0;
if(readExtAddrSPI(IF_ADC0)          != IF_ADC0_VALUE             ) ret=0;
if(readExtAddrSPI(FS_DIG1)          != FS_DIG1_VALUE             ) ret=0;
if(readExtAddrSPI(FS_DIG0)          != FS_DIG0_VALUE             ) ret=0;
if(readExtAddrSPI(FS_CAL3)          != FS_CAL3_VALUE             ) ret=0;
if(readExtAddrSPI(FS_CAL2)          != FS_CAL2_VALUE             ) ret=0;
if(readExtAddrSPI(FS_CAL1)          != FS_CAL1_VALUE             ) ret=0;
if(readExtAddrSPI(FS_CAL0)          != FS_CAL0_VALUE             ) ret=0;
if(readExtAddrSPI(FS_CHP)           != FS_CHP_VALUE              ) ret=0;
if(readExtAddrSPI(FS_DIVTWO)        != FS_DIVTWO_VALUE           ) ret=0;
if(readExtAddrSPI(FS_DSM1)          != FS_DSM1_VALUE             ) ret=0;
if(readExtAddrSPI(FS_DSM0)          != FS_DSM0_VALUE             ) ret=0;
if(readExtAddrSPI(FS_DVC1)          != FS_DVC1_VALUE             ) ret=0;
if(readExtAddrSPI(FS_DVC0)          != FS_DVC0_VALUE             ) ret=0;
if(readExtAddrSPI(FS_LBI)           != FS_LBI_VALUE              ) ret=0;
if(readExtAddrSPI(FS_PFD)           != FS_PFD_VALUE              ) ret=0;
if(readExtAddrSPI(FS_PRE)           != FS_PRE_VALUE              ) ret=0;
if(readExtAddrSPI(FS_REG_DIV_CML)   != FS_REG_DIV_CML_VALUE      ) ret=0;
if(readExtAddrSPI(FS_SPARE)         != FS_SPARE_VALUE            ) ret=0;
if(readExtAddrSPI(FS_VCO4)          != FS_VCO4_VALUE             ) ret=0;
if(readExtAddrSPI(FS_VCO3)          != FS_VCO3_VALUE             ) ret=0;
if(readExtAddrSPI(FS_VCO2)          != FS_VCO2_VALUE             ) ret=0;
if(readExtAddrSPI(FS_VCO1)          != FS_VCO1_VALUE             ) ret=0;
if(readExtAddrSPI(FS_VCO0)          != FS_VCO0_VALUE             ) ret=0;
if(readExtAddrSPI(GBIAS6)           != GBIAS6_VALUE              ) ret=0;
if(readExtAddrSPI(GBIAS5)           != GBIAS5_VALUE              ) ret=0;
if(readExtAddrSPI(GBIAS4)           != GBIAS4_VALUE              ) ret=0;
if(readExtAddrSPI(GBIAS3)           != GBIAS3_VALUE              ) ret=0;
if(readExtAddrSPI(GBIAS2)           != GBIAS2_VALUE              ) ret=0;
if(readExtAddrSPI(GBIAS1)           != GBIAS1_VALUE              ) ret=0;
if(readExtAddrSPI(GBIAS0)           != GBIAS0_VALUE              ) ret=0;
if(readExtAddrSPI(IFAMP)            != IFAMP_VALUE               ) ret=0;
if(readExtAddrSPI(LNA)              != LNA_VALUE                 ) ret=0;
if(readExtAddrSPI(RXMIX)            != RXMIX_VALUE               ) ret=0;
if(readExtAddrSPI(XOSC5)            != XOSC5_VALUE               ) ret=0;
if(readExtAddrSPI(XOSC4)            != XOSC4_VALUE               ) ret=0;
if(readExtAddrSPI(XOSC3)            != XOSC3_VALUE               ) ret=0;
if(readExtAddrSPI(XOSC2)            != XOSC2_VALUE               ) ret=0;
if(readExtAddrSPI(XOSC1)            != XOSC1_VALUE               ) ret=0;
if(readExtAddrSPI(XOSC0)            != XOSC0_VALUE               ) ret=0;
if(readExtAddrSPI(ANALOG_SPARE)     != ANALOG_SPARE_VALUE        ) ret=0;
if(readExtAddrSPI(PA_CFG3)          != PA_CFG3_VALUE             ) ret=0;
if(readExtAddrSPI(WOR_TIME1)        != WOR_TIME1_VALUE           ) ret=0;
if(readExtAddrSPI(WOR_TIME0)        != WOR_TIME0_VALUE           ) ret=0;
if(readExtAddrSPI(WOR_CAPTURE1)     != WOR_CAPTURE1_VALUE        ) ret=0;
if(readExtAddrSPI(WOR_CAPTURE0)     != WOR_CAPTURE0_VALUE        ) ret=0;
if(readExtAddrSPI(BIST)             != BIST_VALUE                ) ret=0;
if(readExtAddrSPI(DCFILTOFFSET_I1)  != DCFILTOFFSET_I1_VALUE     ) ret=0;
if(readExtAddrSPI(DCFILTOFFSET_I0)  != DCFILTOFFSET_I0_VALUE     ) ret=0;
if(readExtAddrSPI(DCFILTOFFSET_Q1)  != DCFILTOFFSET_Q1_VALUE     ) ret=0;
if(readExtAddrSPI(DCFILTOFFSET_Q0)  != DCFILTOFFSET_Q0_VALUE     ) ret=0;
if(readExtAddrSPI(IQIE_I1)          != IQIE_I1_VALUE             ) ret=0;
if(readExtAddrSPI(IQIE_I0)          != IQIE_I0_VALUE             ) ret=0;
if(readExtAddrSPI(IQIE_Q1)          != IQIE_Q1_VALUE             ) ret=0;
if(readExtAddrSPI(IQIE_Q0)          != IQIE_Q0_VALUE             ) ret=0;
if(readExtAddrSPI(RSSI1)            != RSSI1_VALUE               ) ret=0;
if(readExtAddrSPI(RSSI0)            != RSSI0_VALUE               ) ret=0;
if(readExtAddrSPI(MARCSTATE)        != MARCSTATE_VALUE           ) ret=0;
if(readExtAddrSPI(LQI_VAL)          != LQI_VAL_VALUE             ) ret=0;
if(readExtAddrSPI(PQT_SYNC_ERR)     != PQT_SYNC_ERR_VALUE        ) ret=0;
if(readExtAddrSPI(DEM_STATUS)       != DEM_STATUS_VALUE          ) ret=0;
if(readExtAddrSPI(FREQOFF_EST1)     != FREQOFF_EST1_VALUE        ) ret=0;
if(readExtAddrSPI(FREQOFF_EST0)     != FREQOFF_EST0_VALUE        ) ret=0;
if(readExtAddrSPI(AGC_GAIN3)        != AGC_GAIN3_VALUE           ) ret=0;
if(readExtAddrSPI(AGC_GAIN2)        != AGC_GAIN2_VALUE           ) ret=0;
if(readExtAddrSPI(AGC_GAIN1)        != AGC_GAIN1_VALUE           ) ret=0;
if(readExtAddrSPI(AGC_GAIN0)        != AGC_GAIN0_VALUE           ) ret=0;
if(readExtAddrSPI(CFM_RX_DATA_OUT)  != CFM_RX_DATA_OUT_VALUE     ) ret=0;
if(readExtAddrSPI(CFM_TX_DATA_IN)   != CFM_TX_DATA_IN_VALUE      ) ret=0;
if(readExtAddrSPI(ASK_SOFT_RX_DATA) != ASK_SOFT_RX_DATA_VALUE    ) ret=0;
if(readExtAddrSPI(RNDGEN)           != RNDGEN_VALUE              ) ret=0;
if(readExtAddrSPI(MAGN2)            != MAGN2_VALUE               ) ret=0;
if(readExtAddrSPI(MAGN1)            != MAGN1_VALUE               ) ret=0;
if(readExtAddrSPI(MAGN0)            != MAGN0_VALUE               ) ret=0;
if(readExtAddrSPI(ANG1)             != ANG1_VALUE                ) ret=0;
if(readExtAddrSPI(ANG0)             != ANG0_VALUE                ) ret=0;
if(readExtAddrSPI(CHFILT_I2)        != CHFILT_I2_VALUE           ) ret=0;
if(readExtAddrSPI(CHFILT_I1)        != CHFILT_I1_VALUE           ) ret=0;
if(readExtAddrSPI(CHFILT_I0)        != CHFILT_I0_VALUE           ) ret=0;
if(readExtAddrSPI(CHFILT_Q2)        != CHFILT_Q2_VALUE           ) ret=0;
if(readExtAddrSPI(CHFILT_Q1)        != CHFILT_Q1_VALUE           ) ret=0;
if(readExtAddrSPI(CHFILT_Q0)        != CHFILT_Q0_VALUE           ) ret=0;
if(readExtAddrSPI(GPIO_STATUS)      != GPIO_STATUS_VALUE         ) ret=0;
if(readExtAddrSPI(FSCAL_CTRL)       != FSCAL_CTRL_VALUE          ) ret=0;
if(readExtAddrSPI(PHASE_ADJUST)     != PHASE_ADJUST_VALUE        ) ret=0;
if(readExtAddrSPI(PARTNUMBER)       != PARTNUMBER_VALUE          ) ret=0;
if(readExtAddrSPI(PARTVERSION)      != PARTVERSION_VALUE         ) ret=0;
if(readExtAddrSPI(SERIAL_STATUS)    != SERIAL_STATUS_VALUE       ) ret=0;
if(readExtAddrSPI(MODEM_STATUS1)    != MODEM_STATUS1_VALUE       ) ret=0;
if(readExtAddrSPI(MODEM_STATUS0)    != MODEM_STATUS0_VALUE       ) ret=0;
if(readExtAddrSPI(MARC_STATUS1)     != MARC_STATUS1_VALUE        ) ret=0;
if(readExtAddrSPI(MARC_STATUS0)     != MARC_STATUS0_VALUE        ) ret=0;
if(readExtAddrSPI(PA_IFAMP_TEST)    != PA_IFAMP_TEST_VALUE       ) ret=0;
if(readExtAddrSPI(FSRF_TEST)        != FSRF_TEST_VALUE           ) ret=0;
if(readExtAddrSPI(PRE_TEST)         != PRE_TEST_VALUE            ) ret=0;
if(readExtAddrSPI(PRE_OVR)          != PRE_OVR_VALUE             ) ret=0;
if(readExtAddrSPI(ADC_TEST)         != ADC_TEST_VALUE            ) ret=0;
if(readExtAddrSPI(DVC_TEST)         != DVC_TEST_VALUE            ) ret=0;
if(readExtAddrSPI(ATEST)            != ATEST_VALUE               ) ret=0;
if(readExtAddrSPI(ATEST_LVDS)       != ATEST_LVDS_VALUE          ) ret=0;
if(readExtAddrSPI(ATEST_MODE)       != ATEST_MODE_VALUE          ) ret=0;
if(readExtAddrSPI(XOSC_TEST1)       != XOSC_TEST1_VALUE          ) ret=0;
if(readExtAddrSPI(XOSC_TEST0)       != XOSC_TEST0_VALUE          ) ret=0;
if(readExtAddrSPI(RXFIRST)          != RXFIRST_VALUE             ) ret=0;
if(readExtAddrSPI(TXFIRST)          != TXFIRST_VALUE             ) ret=0;
if(readExtAddrSPI(RXLAST)           != RXLAST_VALUE              ) ret=0;
if(readExtAddrSPI(TXLAST)           != TXLAST_VALUE              ) ret=0;
if(readExtAddrSPI(NUM_TXBYTES)      != NUM_TXBYTES_VALUE         ) ret=0;
if(readExtAddrSPI(NUM_RXBYTES)      != NUM_RXBYTES_VALUE         ) ret=0;
if(readExtAddrSPI(FIFO_NUM_TXBYTES) != FIFO_NUM_TXBYTES_VALUE    ) ret=0;
if(readExtAddrSPI(FIFO_NUM_RXBYTES) != FIFO_NUM_RXBYTES_VALUE    ) ret=0;


if(ret == 0) Serial.println("Register values are incorect");


  strobeSPI(SCAL); //Calibrate frequency synthesizer and turn it off
 
  delay(10);
}
 
void FIFOFlush(){
  strobeSPI(SFRX); // Flush the RX FIFO
  delay(1);
  strobeSPI(SFTX); // Flush the TX FIFO
  delay(1);
}
 