#include <Adafruit_INA260.h>

Adafruit_INA260 ina260 = Adafruit_INA260();

void setup() {
  Serial.begin(9600);
  // Wait until serial port is opened
  while (!Serial) {
    delay(10);
  }

  Serial.println("Adafruit INA260 Test");

  if (!ina260.begin()) {
    Serial.println("Couldn't find INA260 chip");
    while (1);
  }
  Serial.println("Time(s), Current (mA), Voltage (mV), Power (mW)");

}

void loop() {
  Serial.print(millis() / 1000);
  Serial.print(", ");
  Serial.print(ina260.readCurrent());
  Serial.print(", ");


  Serial.print(ina260.readBusVoltage());
  Serial.print(", ");

  Serial.println(ina260.readPower());

  delay(5000);
}
