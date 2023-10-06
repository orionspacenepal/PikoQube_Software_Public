// this program gives the feedback of the status of the string that deploys the antenna. 

int feedbackPin = A6;   // select the input pin for the potentiometer


void setup() {
  // declare the ledPin as an OUTPUT:
  //pinMode(ledPin, OUTPUT);
  Serial.begin(9600);
}

void loop() {
  // read the value from the sensor:
  Serial.println(analogRead(feedbackPin));
  delay(500);
  
}
