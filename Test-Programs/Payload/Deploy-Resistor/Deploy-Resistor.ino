// this program gives the feedback of the status of the string that deploys the antenna. 

int deploymentPin = A7;   // select the input pin for the potentiometer


void setup() {
  // declare the ledPin as an OUTPUT:
  //pinMode(ledPin, OUTPUT);
  Serial.begin(9600);
}

void loop() {
  // read the value from the sensor:
  analogWrite(deploymentPin, 1023);
  Serial.println("Deploy ON");
  delay(5000);
  analogWrite(deploymentPin,0);
  Serial.println("Deploy OFF");
  delay(1000);
  
}
