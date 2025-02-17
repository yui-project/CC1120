#define FSK
// #define CW

//SPI Address Space
#define IOCFG3                0x00
#define IOCFG2                0x01
#define IOCFG1                0x02
#define IOCFG0                0x03
#define SYNC3                 0x04
#define SYNC2                 0x05
#define SYNC1                 0x06
#define SYNC0                 0x07
#define SYNC_CFG1             0x08
#define SYNC_CFG0             0x09
#define DEVIATION_M           0x0A
#define MODCFG_DEV_E          0x0B
#define DCFILT_CFG            0x0C
#define PREAMBLE_CFG1         0x0D
#define PREAMBLE_CFG0         0x0E
#define FREQ_IF_CFG           0x0F
#define IQIC                  0x10
#define CHAN_BW               0x11
#define MDMCFG1               0x12
#define MDMCFG0               0x13
#define SYMBOL_RATE2          0x14
#define SYMBOL_RATE1          0x15
#define SYMBOL_RATE0          0x16
#define AGC_REF               0x17
#define AGC_CS_THR            0x18
#define AGC_GAIN_ADJUST       0x19
#define AGC_CFG3              0x1A
#define AGC_CFG2              0x1B
#define AGC_CFG1              0x1C
#define AGC_CFG0              0x1D
#define FIFO_CFG              0x1E
#define DEV_ADDR              0x1F
#define SETTLING_CFG          0x20
#define FS_CFG                0x21
#define WOR_CFG1              0x22
#define WOR_CFG0              0x23
#define WOR_EVENT0_MSB        0x24
#define WOR_EVENT0_LSB        0x25
#define PKT_CFG2              0x26
#define PKT_CFG1              0x27
#define PKT_CFG0              0x28
#define RFEND_CFG1            0x29
#define RFEND_CFG0            0x2A
#define PA_CFG2               0x2B
#define PA_CFG1               0x2C
#define PA_CFG0               0x2D
#define PKT_LEN               0x2E
#define EXT_ADDR              0x2F
#define SRES                  0x30
#define SFSTXON               0x31
#define SXOFF                 0x32
#define SCAL                  0x33
#define SRX                   0x34
#define STX                   0x35
#define SIDLE                 0x36
#define SAFC                  0x37
#define SWOR                  0x38
#define SPWD                  0x39
#define SFRX                  0x3A
#define SFTX                  0x3B
#define SWORRST               0x3C
#define SNOP                  0x3D
#define DIRECT_MEMORY_ACCESS  0x3E
#define TXRX_FIFO             0x3F

//Extended Register Space Mapping
#define IF_MIX_CFG            0x00
#define FREQOFF_CFG           0x01
#define TOC_CFG               0x02
#define MARC_SPARE            0x03
#define ECG_CFG               0x04
#define CFM_DATA_CFG          0x05
#define EXT_CTRL              0x06
#define RCCAL_FINE            0x07
#define RCCAL_COARSE          0x08
#define RCCAL_OFFSET          0x09
#define FREQOFF1              0x0A
#define FREQOFF0              0x0B
#define FREQ2                 0x0C
#define FREQ1                 0x0D
#define FREQ0                 0x0E
#define IF_ADC2               0x0F
#define IF_ADC1               0x10
#define IF_ADC0               0x11
#define FS_DIG1               0x12
#define FS_DIG0               0x13
#define FS_CAL3               0x14
#define FS_CAL2               0x15
#define FS_CAL1               0x16
#define FS_CAL0               0x17
#define FS_CHP                0x18
#define FS_DIVTWO             0x19
#define FS_DSM1               0x1A
#define FS_DSM0               0x1B
#define FS_DVC1               0x1C
#define FS_DVC0               0x1D
#define FS_LBI                0x1E
#define FS_PFD                0x1F
#define FS_PRE                0x20
#define FS_REG_DIV_CML        0x21
#define FS_SPARE              0x22
#define FS_VCO4               0x23
#define FS_VCO3               0x24
#define FS_VCO2               0x25
#define FS_VCO1               0x26
#define FS_VCO0               0x27
#define GBIAS6                0x28
#define GBIAS5                0x29
#define GBIAS4                0x2A
#define GBIAS3                0x2B
#define GBIAS2                0x2C
#define GBIAS1                0x2D
#define GBIAS0                0x2E
#define IFAMP                 0x2F
#define LNA                   0x30
#define RXMIX                 0x31
#define XOSC5                 0x32
#define XOSC4                 0x33
#define XOSC3                 0x34
#define XOSC2                 0x35
#define XOSC1                 0x36
#define XOSC0                 0x37
#define ANALOG_SPARE          0x38
#define PA_CFG3               0x39
//0x3A~0x63 are not used
#define WOR_TIME1             0x64
#define WOR_TIME0             0x65
#define WOR_CAPTURE1          0x66
#define WOR_CAPTURE0          0x67
#define BIST                  0x68
#define DCFILTOFFSET_I1       0x69
#define DCFILTOFFSET_I0       0x6A
#define DCFILTOFFSET_Q1       0x6B
#define DCFILTOFFSET_Q0       0x6C
#define IQIE_I1               0x6D
#define IQIE_I0               0x6E
#define IQIE_Q1               0x6F
#define IQIE_Q0               0x70
#define RSSI1                 0x71
#define RSSI0                 0x72
#define MARCSTATE             0x73
#define LQI_VAL               0x74
#define PQT_SYNC_ERR          0x75
#define DEM_STATUS            0x76
#define FREQOFF_EST1          0x77
#define FREQOFF_EST0          0x78
#define AGC_GAIN3             0x79
#define AGC_GAIN2             0x7A
#define AGC_GAIN1             0x7B
#define AGC_GAIN0             0x7C
#define CFM_RX_DATA_OUT       0x7D
#define CFM_TX_DATA_IN        0x7E
#define ASK_SOFT_RX_DATA      0x7F
#define RNDGEN                0x80
#define MAGN2                 0x81
#define MAGN1                 0x82
#define MAGN0                 0x83
#define ANG1                  0x84
#define ANG0                  0x85
#define CHFILT_I2             0x86
#define CHFILT_I1             0x87
#define CHFILT_I0             0x88
#define CHFILT_Q2             0x89
#define CHFILT_Q1             0x8A
#define CHFILT_Q0             0x8B
#define GPIO_STATUS           0x8C
#define FSCAL_CTRL            0x8D
#define PHASE_ADJUST          0x8E
#define PARTNUMBER            0x8F
#define PARTVERSION           0x90
#define SERIAL_STATUS         0x91
#define MODEM_STATUS1         0x92
#define MODEM_STATUS0         0x93
#define MARC_STATUS1          0x94
#define MARC_STATUS0          0x95
#define PA_IFAMP_TEST         0x96
#define FSRF_TEST             0x97
#define PRE_TEST              0x98
#define PRE_OVR               0x99
#define ADC_TEST              0x9A
#define DVC_TEST              0x9B
#define ATEST                 0x9C
#define ATEST_LVDS            0x9D
#define ATEST_MODE            0x9E
#define XOSC_TEST1            0x9F
#define XOSC_TEST0            0xA0
//0xA1~0xD1 are not used
#define RXFIRST               0xD2
#define TXFIRST               0xD3
#define RXLAST                0xD4
#define TXLAST                0xD5
#define NUM_TXBYTES           0xD6
#define NUM_RXBYTES           0xD7
#define FIFO_NUM_TXBYTES      0xD8
#define FIFO_NUM_RXBYTES      0xD9

#ifdef CW
  // Register values
  #define IOCFG3_VALUE            0xB0
  #define IOCFG2_VALUE            0x06
  #define IOCFG1_VALUE            0xB0
  #define IOCFG0_VALUE            0x40
  #define SYNC3_VALUE             0x93
  #define SYNC2_VALUE             0x0B
  #define SYNC1_VALUE             0x51
  #define SYNC0_VALUE             0xDE
  #define SYNC_CFG1_VALUE         0x0B
  #define SYNC_CFG0_VALUE         0x17
  #define DEVIATION_M_VALUE       0x89
  #define MODCFG_DEV_E_VALUE      0x19
  #define DCFILT_CFG_VALUE        0x1C
  #define PREAMBLE_CFG1_VALUE     0x18
  #define PREAMBLE_CFG0_VALUE     0x2A
  #define FREQ_IF_CFG_VALUE       0x40
  #define IQIC_VALUE              0xC6
  #define CHAN_BW_VALUE           0x08
  #define MDMCFG1_VALUE           0x46
  #define MDMCFG0_VALUE           0x05
  #define SYMBOL_RATE2_VALUE      0x48
  #define SYMBOL_RATE1_VALUE      0x93
  #define SYMBOL_RATE0_VALUE      0x75
  #define AGC_REF_VALUE           0x20
  #define AGC_CS_THR_VALUE        0x19
  #define AGC_GAIN_ADJUST_VALUE   0x00
  #define AGC_CFG3_VALUE          0x91
  #define AGC_CFG2_VALUE          0x20
  #define AGC_CFG1_VALUE          0xA9
  #define AGC_CFG0_VALUE          0xC3
  #define FIFO_CFG_VALUE          0x00
  #define DEV_ADDR_VALUE          0x00
  #define SETTLING_CFG_VALUE      0x0B
  #define FS_CFG_VALUE            0x14
  #define WOR_CFG1_VALUE          0x08
  #define WOR_CFG0_VALUE          0x21
  #define WOR_EVENT0_MSB_VALUE    0x00
  #define WOR_EVENT0_LSB_VALUE    0x00
  #define PKT_CFG2_VALUE          0x04
  #define PKT_CFG1_VALUE          0x15
  #define PKT_CFG0_VALUE          0x20
  #define RFEND_CFG1_VALUE        0x0F
  #define RFEND_CFG0_VALUE        0x00
  #define PA_CFG2_VALUE           0x7C
  #define PA_CFG1_VALUE           0x56
  #define PA_CFG0_VALUE           0x7E
  #define PKT_LEN_VALUE           0xFF
  #define IF_MIX_CFG_VALUE        0x00
  #define FREQOFF_CFG_VALUE       0x22
  #define TOC_CFG_VALUE           0x0B
  #define MARC_SPARE_VALUE        0x00
  #define ECG_CFG_VALUE           0x00
  #define CFM_DATA_CFG_VALUE      0x00
  #define EXT_CTRL_VALUE          0x01
  #define RCCAL_FINE_VALUE        0x00
  #define RCCAL_COARSE_VALUE      0x00
  #define RCCAL_OFFSET_VALUE      0x00
  #define FREQOFF1_VALUE          0x00
  #define FREQOFF0_VALUE          0x00
  #define FREQ2_VALUE             0x6D
  #define FREQ1_VALUE             0x43
  #define FREQ0_VALUE             0x33
  #define IF_ADC2_VALUE           0x02
  #define IF_ADC1_VALUE           0xA6
  #define IF_ADC0_VALUE           0x04
  #define FS_DIG1_VALUE           0x00
  #define FS_DIG0_VALUE           0x5F
  #define FS_CAL3_VALUE           0x00
  #define FS_CAL2_VALUE           0x20
  #define FS_CAL1_VALUE           0x40
  #define FS_CAL0_VALUE           0x0E
  #define FS_CHP_VALUE            0x28
  #define FS_DIVTWO_VALUE         0x03
  #define FS_DSM1_VALUE           0x00
  #define FS_DSM0_VALUE           0x33
  #define FS_DVC1_VALUE           0xFF
  #define FS_DVC0_VALUE           0x17
  #define FS_LBI_VALUE            0x00
  #define FS_PFD_VALUE            0x50
  #define FS_PRE_VALUE            0x6E
  #define FS_REG_DIV_CML_VALUE    0x14
  #define FS_SPARE_VALUE          0xAC
  #define FS_VCO4_VALUE           0x14
  #define FS_VCO3_VALUE           0x00
  #define FS_VCO2_VALUE           0x00
  #define FS_VCO1_VALUE           0x00
  #define FS_VCO0_VALUE           0xB4
  #define GBIAS6_VALUE            0x00
  #define GBIAS5_VALUE            0x02
  #define GBIAS4_VALUE            0x00
  #define GBIAS3_VALUE            0x00
  #define GBIAS2_VALUE            0x10
  #define GBIAS1_VALUE            0x00
  #define GBIAS0_VALUE            0x00
  #define IFAMP_VALUE             0x01
  #define LNA_VALUE               0x01
  #define RXMIX_VALUE             0x01
  #define XOSC5_VALUE             0x0E
  #define XOSC4_VALUE             0xA0
  #define XOSC3_VALUE             0x03
  #define XOSC2_VALUE             0x04
  #define XOSC1_VALUE             0x03
  #define XOSC0_VALUE             0x00
  #define ANALOG_SPARE_VALUE      0x00
  #define PA_CFG3_VALUE           0x00
  #define WOR_TIME1_VALUE         0x00
  #define WOR_TIME0_VALUE         0x00
  #define WOR_CAPTURE1_VALUE      0x00
  #define WOR_CAPTURE0_VALUE      0x00
  #define BIST_VALUE              0x00
  #define DCFILTOFFSET_I1_VALUE   0x00
  #define DCFILTOFFSET_I0_VALUE   0x00
  #define DCFILTOFFSET_Q1_VALUE   0x00
  #define DCFILTOFFSET_Q0_VALUE   0x00
  #define IQIE_I1_VALUE           0x00
  #define IQIE_I0_VALUE           0x00
  #define IQIE_Q1_VALUE           0x00
  #define IQIE_Q0_VALUE           0x00
  #define RSSI1_VALUE             0x80
  #define RSSI0_VALUE             0x00
  #define MARCSTATE_VALUE         0x41
  #define LQI_VAL_VALUE           0x00
  #define PQT_SYNC_ERR_VALUE      0xFF
  #define DEM_STATUS_VALUE        0x00
  #define FREQOFF_EST1_VALUE      0x00
  #define FREQOFF_EST0_VALUE      0x00
  #define AGC_GAIN3_VALUE         0x00
  #define AGC_GAIN2_VALUE         0xD1
  #define AGC_GAIN1_VALUE         0x00
  #define AGC_GAIN0_VALUE         0x3F
  #define CFM_RX_DATA_OUT_VALUE   0x00
  #define CFM_TX_DATA_IN_VALUE    0x00
  #define ASK_SOFT_RX_DATA_VALUE  0x30
  #define RNDGEN_VALUE            0x7F
  #define MAGN2_VALUE             0x00
  #define MAGN1_VALUE             0x00
  #define MAGN0_VALUE             0x00
  #define ANG1_VALUE              0x00
  #define ANG0_VALUE              0x00
  #define CHFILT_I2_VALUE         0x08
  #define CHFILT_I1_VALUE         0x00
  #define CHFILT_I0_VALUE         0x00
  #define CHFILT_Q2_VALUE         0x00
  #define CHFILT_Q1_VALUE         0x00
  #define CHFILT_Q0_VALUE         0x00
  #define GPIO_STATUS_VALUE       0x00
  #define FSCAL_CTRL_VALUE        0x01
  #define PHASE_ADJUST_VALUE      0x00
  #define PARTNUMBER_VALUE        0x48
  #define PARTVERSION_VALUE       0x23
  #define SERIAL_STATUS_VALUE     0x00
  #define MODEM_STATUS1_VALUE     0x10
  #define MODEM_STATUS0_VALUE     0x00
  #define MARC_STATUS1_VALUE      0x00
  #define MARC_STATUS0_VALUE      0x00
  #define PA_IFAMP_TEST_VALUE     0x00
  #define FSRF_TEST_VALUE         0x00
  #define PRE_TEST_VALUE          0x00
  #define PRE_OVR_VALUE           0x00
  #define ADC_TEST_VALUE          0x00
  #define DVC_TEST_VALUE          0x0B
  #define ATEST_VALUE             0x40
  #define ATEST_LVDS_VALUE        0x00
  #define ATEST_MODE_VALUE        0x00
  #define XOSC_TEST1_VALUE        0x3C
  #define XOSC_TEST0_VALUE        0x00
  #define RXFIRST_VALUE           0x00
  #define TXFIRST_VALUE           0x00
  #define RXLAST_VALUE            0x00
  #define TXLAST_VALUE            0x00
  #define NUM_TXBYTES_VALUE       0x00
  #define NUM_RXBYTES_VALUE       0x00
  #define FIFO_NUM_TXBYTES_VALUE  0x0F
  #define FIFO_NUM_RXBYTES_VALUE  0x00
#endif

#ifdef FSK
// Register values
#define IOCFG3_VALUE            0xB0
#define IOCFG2_VALUE            0x06
#define IOCFG1_VALUE            0xB0
#define IOCFG0_VALUE            0x40
#define SYNC3_VALUE             0x93
#define SYNC2_VALUE             0x0B
#define SYNC1_VALUE             0x51
#define SYNC0_VALUE             0xDE
#define SYNC_CFG1_VALUE         0x0B
#define SYNC_CFG0_VALUE         0x17
#define DEVIATION_M_VALUE       0x48
#define MODCFG_DEV_E_VALUE      0x04
#define DCFILT_CFG_VALUE        0x1C
#define PREAMBLE_CFG1_VALUE     0x18
#define PREAMBLE_CFG0_VALUE     0x2A
#define FREQ_IF_CFG_VALUE       0x40
#define IQIC_VALUE              0x46
#define CHAN_BW_VALUE           0x01
#define MDMCFG1_VALUE           0x46
#define MDMCFG0_VALUE           0x05
#define SYMBOL_RATE2_VALUE      0xA9
#define SYMBOL_RATE1_VALUE      0x99
#define SYMBOL_RATE0_VALUE      0x9A
#define AGC_REF_VALUE           0x20
#define AGC_CS_THR_VALUE        0x19
#define AGC_GAIN_ADJUST_VALUE   0x00
#define AGC_CFG3_VALUE          0x91
#define AGC_CFG2_VALUE          0x20
#define AGC_CFG1_VALUE          0xA9
#define AGC_CFG0_VALUE          0xCF
#define FIFO_CFG_VALUE          0x00
#define DEV_ADDR_VALUE          0x00
#define SETTLING_CFG_VALUE      0x03
#define FS_CFG_VALUE            0x14
#define WOR_CFG1_VALUE          0x08
#define WOR_CFG0_VALUE          0x21
#define WOR_EVENT0_MSB_VALUE    0x00
#define WOR_EVENT0_LSB_VALUE    0x00
#define PKT_CFG2_VALUE          0x04
#define PKT_CFG1_VALUE          0x05
#define PKT_CFG0_VALUE          0x20
#define RFEND_CFG1_VALUE        0x0F
#define RFEND_CFG0_VALUE        0x00
#define PA_CFG2_VALUE           0x7F
#define PA_CFG1_VALUE           0x56
#define PA_CFG0_VALUE           0x7A
#define PKT_LEN_VALUE           0xFF
#define IF_MIX_CFG_VALUE        0x00
#define FREQOFF_CFG_VALUE       0x22
#define TOC_CFG_VALUE           0x0B
#define MARC_SPARE_VALUE        0x00
#define ECG_CFG_VALUE           0x00
#define CFM_DATA_CFG_VALUE      0x00
#define EXT_CTRL_VALUE          0x01
#define RCCAL_FINE_VALUE        0x00
#define RCCAL_COARSE_VALUE      0x00
#define RCCAL_OFFSET_VALUE      0x00
#define FREQOFF1_VALUE          0x00
#define FREQOFF0_VALUE          0x00
#define FREQ2_VALUE             0x6D
#define FREQ1_VALUE             0x43
#define FREQ0_VALUE             0x33
#define IF_ADC2_VALUE           0x02
#define IF_ADC1_VALUE           0xA6
#define IF_ADC0_VALUE           0x04
#define FS_DIG1_VALUE           0x00
#define FS_DIG0_VALUE           0x5F
#define FS_CAL3_VALUE           0x00
#define FS_CAL2_VALUE           0x20
#define FS_CAL1_VALUE           0x40
#define FS_CAL0_VALUE           0x0E
#define FS_CHP_VALUE            0x28
#define FS_DIVTWO_VALUE         0x03
#define FS_DSM1_VALUE           0x00
#define FS_DSM0_VALUE           0x33
#define FS_DVC1_VALUE           0xFF
#define FS_DVC0_VALUE           0x17
#define FS_LBI_VALUE            0x00
#define FS_PFD_VALUE            0x50
#define FS_PRE_VALUE            0x6E
#define FS_REG_DIV_CML_VALUE    0x14
#define FS_SPARE_VALUE          0xAC
#define FS_VCO4_VALUE           0x14
#define FS_VCO3_VALUE           0x00
#define FS_VCO2_VALUE           0x00
#define FS_VCO1_VALUE           0x00
#define FS_VCO0_VALUE           0xB4
#define GBIAS6_VALUE            0x00
#define GBIAS5_VALUE            0x02
#define GBIAS4_VALUE            0x00
#define GBIAS3_VALUE            0x00
#define GBIAS2_VALUE            0x10
#define GBIAS1_VALUE            0x00
#define GBIAS0_VALUE            0x00
#define IFAMP_VALUE             0x01
#define LNA_VALUE               0x01
#define RXMIX_VALUE             0x01
#define XOSC5_VALUE             0x0E
#define XOSC4_VALUE             0xA0
#define XOSC3_VALUE             0x03
#define XOSC2_VALUE             0x04
#define XOSC1_VALUE             0x03
#define XOSC0_VALUE             0x00
#define ANALOG_SPARE_VALUE      0x00
#define PA_CFG3_VALUE           0x00
#define WOR_TIME1_VALUE         0x00
#define WOR_TIME0_VALUE         0x00
#define WOR_CAPTURE1_VALUE      0x00
#define WOR_CAPTURE0_VALUE      0x00
#define BIST_VALUE              0x00
#define DCFILTOFFSET_I1_VALUE   0x00
#define DCFILTOFFSET_I0_VALUE   0x00
#define DCFILTOFFSET_Q1_VALUE   0x00
#define DCFILTOFFSET_Q0_VALUE   0x00
#define IQIE_I1_VALUE           0x00
#define IQIE_I0_VALUE           0x00
#define IQIE_Q1_VALUE           0x00
#define IQIE_Q0_VALUE           0x00
#define RSSI1_VALUE             0x80
#define RSSI0_VALUE             0x00
#define MARCSTATE_VALUE         0x41
#define LQI_VAL_VALUE           0x00
#define PQT_SYNC_ERR_VALUE      0xFF
#define DEM_STATUS_VALUE        0x00
#define FREQOFF_EST1_VALUE      0x00
#define FREQOFF_EST0_VALUE      0x00
#define AGC_GAIN3_VALUE         0x00
#define AGC_GAIN2_VALUE         0xD1
#define AGC_GAIN1_VALUE         0x00
#define AGC_GAIN0_VALUE         0x3F
#define CFM_RX_DATA_OUT_VALUE   0x00
#define CFM_TX_DATA_IN_VALUE    0x00
#define ASK_SOFT_RX_DATA_VALUE  0x30
#define RNDGEN_VALUE            0x7F
#define MAGN2_VALUE             0x00
#define MAGN1_VALUE             0x00
#define MAGN0_VALUE             0x00
#define ANG1_VALUE              0x00
#define ANG0_VALUE              0x00
#define CHFILT_I2_VALUE         0x08
#define CHFILT_I1_VALUE         0x00
#define CHFILT_I0_VALUE         0x00
#define CHFILT_Q2_VALUE         0x00
#define CHFILT_Q1_VALUE         0x00
#define CHFILT_Q0_VALUE         0x00
#define GPIO_STATUS_VALUE       0x00
#define FSCAL_CTRL_VALUE        0x01
#define PHASE_ADJUST_VALUE      0x00
#define PARTNUMBER_VALUE        0x48
#define PARTVERSION_VALUE       0x21
#define SERIAL_STATUS_VALUE     0x00
#define MODEM_STATUS1_VALUE     0x10
#define MODEM_STATUS0_VALUE     0x00
#define MARC_STATUS1_VALUE      0x00
#define MARC_STATUS0_VALUE      0x00
#define PA_IFAMP_TEST_VALUE     0x00
#define FSRF_TEST_VALUE         0x00
#define PRE_TEST_VALUE          0x00
#define PRE_OVR_VALUE           0x00
#define ADC_TEST_VALUE          0x00
#define DVC_TEST_VALUE          0x0B
#define ATEST_VALUE             0x40
#define ATEST_LVDS_VALUE        0x00
#define ATEST_MODE_VALUE        0x00
#define XOSC_TEST1_VALUE        0x3C
#define XOSC_TEST0_VALUE        0x00
#define RXFIRST_VALUE           0x00
#define TXFIRST_VALUE           0x00
#define RXLAST_VALUE            0x00
#define TXLAST_VALUE            0x00
#define NUM_TXBYTES_VALUE       0x00
#define NUM_RXBYTES_VALUE       0x00
#define FIFO_NUM_TXBYTES_VALUE  0x0F
#define FIFO_NUM_RXBYTES_VALUE  0x00

#endif