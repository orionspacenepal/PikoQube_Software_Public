//******************************************************************************** PQ Training Kit, ORION Space *********************************************************************************************
//******************************************************************************** EEPROM Test     *********************************************************************************************
//This program is an example to test EEPROM storage for permanently storing the critical data. 
// Concept of EEPROM: This concept can be applied to store data permanently. Example: Storing the status of antenna deplyment. Antenna deployment is a one time run command until and unless any override is done.
// The EEPROM memory has a specified life of 100,000 write/erase cycles. The capacity is 1024 bytes in Atmega328p

#include <EEPROM.h>   //Adding EEPROM header file

int EEPROMAddress=121;  //Address of EEPROM to sore the data
int EEPROMvalue=3;  //Value to store. Maximum is 255.

void setup() {
  Serial.begin(9600);
  EEPROM.write(EEPROMAddress,EEPROMvalue);  //Writing to EEPROM
}

void loop() {
 Serial.print("EEPROM value: ");
 Serial.println(EEPROM.read(EEPROMAddress));  //Reading from EEPROM
}
