//******************************************************************************** PQ Training Kit, ORION Space *********************************************************************************************
//******************************************************************************** Watchdog Reset time Test     *********************************************************************************************
//This program is an example to test Watchdog Reset timeout period. 
//The timeout is controlled using three DIP switches(SW3) connected to OBC of PQ Training Kit.
//The OBC is restarted once the timeout is reached. To avoid this, remove comment 'toggleWDI()'
// Concept of Watchdog: If the input in Watchdog Input Pin(WDI) is not toggled 
//within the watchdog restart time, Watchdog generates LOW signal at Watchdog Output Pin (WDO)
// This concept can be used to avoid satellite from Single Event Upset.

int Led=13;
int WDIpin=8;  //Watchdog Input Pin
boolean WDIStatus=HIGH; //Storing status of Toggle Pin

void setup() {
 Serial.begin(9600);
 pinMode(Led,OUTPUT);
 pinMode(WDIpin,OUTPUT);

}
void loop() {
 //toggleWDI();   //Uncomment this line to avoid restart time out
digitalWrite(Led,HIGH);  
Serial.println(millis()/1000);

}

void toggleWDI()
{
  //This function toggles the Watchdog Input Pin
  WDIStatus=!WDIStatus;
  digitalWrite(WDIpin,WDIStatus);
}
