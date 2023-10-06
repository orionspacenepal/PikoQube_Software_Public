//Reading temperature value using BMP180
#include <SFE_BMP180.h>
#include <Wire.h>

SFE_BMP180 pressure;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  if (pressure.begin()) {
    Serial.println("Pressure sensor is working");
  }
  else {
    Serial.println("Something is wrong with BMP180");
    while (1);
  }
}

void loop() {
  double T;
  char status;

  status = pressure.startTemperature();
  //the starting the request is unsuccessful, 0 is returned
  if (status != 0) {

    //the function gettemperature returns 1 if successfull and 0 if unsuccessfull
    //the temperature value is stored in variable T
    //the unit of temperature is deg C by default
    
    status = pressure.getTemperature(T);
    Serial.print("Temperature is celcius: ");
    Serial.println(T);
    

    Serial.print("Temperature in F: ");
    Serial.println((1.8*T)+32);

    delay(1000);    

  }

}
