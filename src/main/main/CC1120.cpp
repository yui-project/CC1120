#include "CC1120.h"
#include <SPI.h>

uint8_t CC1120Class::readSPI(uint8_t addr) {
  digitalWrite(CC1120_SS_PIN, LOW);
  CC1120_SPI.transfer(CC1120_R_BIT | addr);
  uint8_t v = CC1120_SPI.transfer(0x00);
  digitalWrite(CC1120_SS_PIN, HIGH);
  return v;
}
 
void CC1120Class::writeSPI(uint8_t addr, uint8_t value) {
  digitalWrite(CC1120_SS_PIN, LOW);
  CC1120_SPI.transfer(addr);
  CC1120_SPI.transfer(value);
  digitalWrite(CC1120_SS_PIN, HIGH);
}
 
void CC1120Class::strobeSPI(uint8_t cmd)
{
  digitalWrite(CC1120_SS_PIN, LOW);
  CC1120_SPI.transfer(CC1120_R_BIT | cmd);
  digitalWrite(CC1120_SS_PIN, HIGH);
}
 
uint8_t CC1120Class::readExtAddrSPI(uint8_t addr) {
  static uint8_t v;
  // CC1120_SPI.beginTransaction(settings);
  digitalWrite(CC1120_SS_PIN, LOW);
  CC1120_SPI.transfer(CC1120_R_BIT | EXT_ADDR);
  CC1120_SPI.transfer(addr);
  delayMicroseconds(10);
  v = CC1120_SPI.transfer(0xff);
  // Serial.println(CC1120_SPI.transfer(0xff), BIN);
  digitalWrite(CC1120_SS_PIN, HIGH);
  CC1120_SPI.endTransaction();
  return v;
}
 
void CC1120Class::writeExtAddrSPI(uint8_t addr, uint8_t value) {
  int v;
  digitalWrite(CC1120_SS_PIN, LOW);
  v = CC1120_SPI.transfer(EXT_ADDR);
  v = CC1120_SPI.transfer(addr);
  v = CC1120_SPI.transfer(value);
  digitalWrite(CC1120_SS_PIN, HIGH);
}



void CC1120Class::timer_start(uint8_t time)
{
  timerTime = millis() + time;
}

bool CC1120Class::timeout()
{
  if(timerTime == millis()){
    return 1;
  }
  return 0;
}

void CC1120Class::FIFOFlush(){
  strobeSPI(SFRX); // Flush the RX FIFO
  strobeSPI(SFTX); // Flush the TX FIFO
}

// return value{1: success, 0: SPI R/W error, -1: calibration error}
int8_t CC1120Class::begin()
{
  int8_t ret = 1;

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


  if(0xB0 != readSPI(IOCFG3)        ) ret = 0;
  if(0x06 != readSPI(IOCFG2)        ) ret = 0;
  if(0xB0 != readSPI(IOCFG1)        ) ret = 0;
  if(0x40 != readSPI(IOCFG0)        ) ret = 0;
  if(0x0B != readSPI(SYNC_CFG1)     ) ret = 0;
  if(0x89 != readSPI(DEVIATION_M)   ) ret = 0;
  if(0x01 != readSPI(MODCFG_DEV_E)  ) ret = 0;
  if(0x1C != readSPI(DCFILT_CFG)    ) ret = 0;
  if(0x18 != readSPI(PREAMBLE_CFG1) ) ret = 0;
  if(0xC6 != readSPI(IQIC)          ) ret = 0;
  if(0x08 != readSPI(CHAN_BW)       ) ret = 0;
  if(0x05 != readSPI(MDMCFG0)       ) ret = 0;
  if(0x48 != readSPI(SYMBOL_RATE2)  ) ret = 0;
  if(0x93 != readSPI(SYMBOL_RATE1)  ) ret = 0;
  if(0x75 != readSPI(SYMBOL_RATE0)  ) ret = 0;
  if(0x20 != readSPI(AGC_REF)       ) ret = 0;
  if(0x19 != readSPI(AGC_CS_THR)    ) ret = 0;
  if(0xA9 != readSPI(AGC_CFG1)      ) ret = 0;
  if(0xCF != readSPI(AGC_CFG0)      ) ret = 0;
  if(0x00 != readSPI(FIFO_CFG)      ) ret = 0;
  if(0x14 != readSPI(FS_CFG)        ) ret = 0;
  if(0x20 != readSPI(PKT_CFG0)      ) ret = 0;
  if(0x7E != readSPI(PA_CFG0)       ) ret = 0;
  if(0xFF != readSPI(PKT_LEN)       ) ret = 0;
  if(0x00 != readSPI(IF_MIX_CFG)    ) ret = 0;
  if(0x22 != readExtAddrSPI(FREQOFF_CFG)   ) ret = 0;
  if(0x6D != readExtAddrSPI(FREQ2)         ) ret = 0;
  if(0x43 != readExtAddrSPI(FREQ1)         ) ret = 0;
  if(0x33 != readExtAddrSPI(FREQ0)         ) ret = 0;
  if(0x00 != readExtAddrSPI(FS_DIG1)       ) ret = 0;
  if(0x5F != readExtAddrSPI(FS_DIG0)       ) ret = 0;
  if(0x40 != readExtAddrSPI(FS_CAL1)       ) ret = 0;
  if(0x0E != readExtAddrSPI(FS_CAL0)       ) ret = 0;
  if(0x03 != readExtAddrSPI(FS_DIVTWO)     ) ret = 0;
  if(0x33 != readExtAddrSPI(FS_DSM0)       ) ret = 0;
  if(0x17 != readExtAddrSPI(FS_DVC0)       ) ret = 0;
  if(0x50 != readExtAddrSPI(FS_PFD)        ) ret = 0;
  if(0x6E != readExtAddrSPI(FS_PRE)        ) ret = 0;
  if(0x14 != readExtAddrSPI(FS_REG_DIV_CML)) ret = 0;
  if(0xAC != readExtAddrSPI(FS_SPARE)      ) ret = 0;
  if(0xB4 != readExtAddrSPI(FS_VCO0)       ) ret = 0;
  if(0x0E != readExtAddrSPI(XOSC5)         ) ret = 0;
  if(0x03 != readExtAddrSPI(XOSC1)         ) ret = 0;
  if(0x48 != readExtAddrSPI(PARTNUMBER)    ) ret = 0;
  if(0x23 != readExtAddrSPI(PARTVERSION)   ) ret = 0;
  if(0x10 != readExtAddrSPI(MODEM_STATUS1) ) ret = 0;
  
  strobeSPI(SCAL); //Calibrate frequency synthesizer and turn it off

  timer_start(timerTime);

  do
  {
    if(timeout())
    {
      ret = -1;
      break;
    }
  } while(readExtAddrSPI(MARCSTATE) != 0b00100001);

  return ret;
}

bool CC1120Class::cal()
{
  bool ret = 1;

  strobeSPI(SCAL); //Calibrate frequency synthesizer and turn it off

  timer_start(timerTime);

  do
  {
    if(timeout())
    {
      ret = 0;
      break;
    }
  } while(readExtAddrSPI(MARCSTATE) != 0b00100001);

  return ret;
}

int8_t CC1120Class::TX(uint32_t *payload, uint16_t len)
{
  int8_t ret = 1;
  uint32_t remaining_size = len;

  if(len < 128)
  {
    for(uint32_t i=0; i<128; i++)
    {
      writeSPI(TXRX_FIFO, payload[i]);
    }
    strobeSPI(STX);
  }
  else if(len > 128)
  {
    uint32_t index = 0;

    for(uint32_t i=0; i<128; i++)
      {
        writeSPI(TXRX_FIFO, payload[index++]);
      }

    strobeSPI(STX);

    do
    {
      if(readExtAddrSPI(FIFO_NUM_TXBYTES) < 15)
      {
        if(remaining_size > 64)
        {
          for(int i=0; i<64; i++)
          {
            writeSPI(TXRX_FIFO, payload[index++]);
          }
          remaining_size -= 64;
        }

        else if (remaining_size > 0)
        {
          for(int i=0; i<remaining_size; i++)
          {
            writeSPI(TXRX_FIFO, payload[index++]);
          }
          remaining_size = 0;
        }
      }
    } while(remaining_size > 0);   

  }

  timer_start(250);
  while(readExtAddrSPI(MARCSTATE) != 0b00010110){
    if(timeout())
    {
      ret = 0;
    }
  }

  strobeSPI(SIDLE);

  timer_start(250);
  while(readExtAddrSPI(MARCSTATE) != 0b00110011){
    if(timeout())
    {
      goto END; //go to end of this function
    }
  }

  END:
  FIFOFlush();
  return let;
}

uint8_t CC1120Class::*RX()
{
  strobeSPI(SRX);

  timer_start(250);
  while(readExtAddrSPI(MARCSTATE) != 0b01101101){
    if(timeout())
    {
      goto END; //go to end of this function
    }
  }

  uint8_t RXByte = 0;
  timer_start(250);
  while(RXByte < 1){
    RXByte = readExtAddrSPI(FIFO_NUM_RXBYTES);
    if(timeout())
    {
      goto END; //go to end of this function
    }
  }

  uint8_t DataLen = 0;
  DataLen = readSPI(0b10111111);

  uint8_t RXData[DataLen];
  if(DataLen < 127)
  {
    for(int i=0; i<DataLen; i++)
    {
      RxData[i] = readSPI(0b10111111);
    }
  }
  else if(DataLen > 127)
  {
    uint32_t index = 0;
    while(DataLen > 0)
    {
      if(DataLen >= 127)
      {
        for(int i=0; i<127; i++)
        {
          RxData[index++] = readSPI(0b10111111);
        }
        DataLen -= 127;
      }
      else if(DataLen < 127)
      {
        for(int i=0; i<DataLen; i++)
        {
          RxData[index++] = readSPI(0b10111111);
        }
        DataLen = 0;
      }
    }
  }
  strobeSPI(SIDLE);

  timer_start(250);
  while(readExtAddrSPI(MARCSTATE) != 0b00110011){
    if(timeout())
    {
      goto END; //go to end of this function
    }
  }

  END:
  FIFOFlush();

  return RxData;
}