#include <SPI.h>
#include "CC1120.h"
 
#define R_BIT           0x80
#define SS_PIN          4

CC1120Class CC1120;

void setup() {
  // put your setup code here, to run once:
  CC1120.begin();
}

void loop() {
  // put your main code here, to run repeatedly:
  delay(1);
}
