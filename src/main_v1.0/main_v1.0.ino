#include <SPI.h>
#include "CC1120.h"
 
#define R_BIT           0x80
#define SS_PIN          4

CC1120Class CC1120;

bool ret = 1;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  // Serial.println(ret);
  ret = CC1120.begin();
  Serial.print("after begin()");
  Serial.println(ret);
}

void loop() {
  // put your main code here, to run repeatedly:
  // Serial.println("loop");
  // uint8_t payload[126]= "In twilight,s glow, the silence calls, a whisper drifting, soft and small. The wind, it dances, wild and free, a song of time,";
  // ret = CC1120.TX(payload, 124);
  // Serial.println(ret);
  // delay(5000);

  uint8_t data[28];
  ret = CC1120.RX(data, 28);
  if(ret == 1){
    for(int i=0; i<28; i++) Serial.println((char)data[i]);
  }
  delay(1000);
}
