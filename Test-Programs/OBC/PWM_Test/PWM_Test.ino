//******************************************************************************** PQ Training Kit, ORION Space *********************************************************************************************
//******************************************************************************** PWM Test Source Code         *********************************************************************************************
// Generating PWM in digital output pin 5 which is connected to D7 of PQ Training Kit v1
// Note: Not all pins can generate PWM. Please check PWM pins of PQ training kit as well as PWM pins of arduino.


int LedPWM = 5;           // the PWM pin the LED is attached to D7 of PQ Training Kit 1.0 . 
int Brightness = 0;      // how bright the LED is
int FadeIncrement = 25;   // how many points to fade the LED by

void setup() {
  // Start by Defining the pin you are using as an OUTPUT
  pinMode(LedPWM, OUTPUT);
}
void loop() {
  // Using AnalogWrite to generate PWM based analog voltage. Check "analogWrite(pin,value)" function for more details. https://www.arduino.cc/reference/en/language/functions/analog-io/analogwrite/
  analogWrite(LedPWM, Brightness);
  Brightness = Brightness + FadeIncrement;  //Increase the brightness by FadeIncrement Amount
 if (Brightness <= 0 || Brightness >= 255) { // The resolution of analogWrite is 8 bits by default. Therefore, the maximum value analogWrite function accepts is 255. 
    FadeIncrement = -FadeIncrement;  //Decrease the brightness once maximum value is reached.
  }
  delay(30);
}
