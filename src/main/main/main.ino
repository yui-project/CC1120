#include <SPI.h>
#include "CC1120.h"
 
#define R_BIT           0x80
#define SS_PIN          4

void setup() {
  // put your setup code here, to run once:
  
  CC1120_SPI.begin();
  SPISettings settings(1000000, MSBFIRST, SPI_MODE0);
}

void loop() {
  // put your main code here, to run repeatedly:

}
