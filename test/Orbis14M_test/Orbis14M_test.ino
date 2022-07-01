#include "RLS_Orbis14M.h"

#define SCK            18
#define MISO           19
#define MOSI           23
#define SS             5

#define RATIO_MOTOR   50


RLS_Orbis14M encoder;

void setup() {
  Serial.begin(115200);
  if (!encoder.begin(SCK, MISO, MOSI, SS)) {
	  Serial.println("Could not find a valid OrbisM14 sensor, check wiring!");
	  while (1) {}
  }
}

void loop() 
{
  uint8_t ratio_motor = RATIO_MOTOR;
  uint8_t status;
  uint16_t counter_value = 0;
  float angle;
  float temp;

  delay(5000);

  status = encoder.readStatus();
  Serial.println(status, BIN);
  delay(1000);

  angle = encoder.readAngularPosition(ratio_motor, true);
  Serial.print("Angle before: ");
  Serial.println(angle);
  
  encoder.setPositionOffsetToZero();
  delay(1000);
  encoder.setCounterValue(counter_value);
  

  angle = encoder.readAngularPosition(ratio_motor, true);
  Serial.print("Angle after: ");
  Serial.println(angle);

  while(1)
  {
    delay(5000);

    temp = encoder.readTemperature();
    Serial.println("/*********************************/");
    Serial.print("Temperature: ");
    Serial.println(temp/10);
    Serial.println("/*********************************/");


    angle = encoder.readAngularPosition(ratio_motor, true);
    Serial.print("Angle: ");
    Serial.println(angle);

  }
}