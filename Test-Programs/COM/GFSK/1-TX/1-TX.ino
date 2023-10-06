#include <SPI.h>
#include <RH_RF24.h>

// Singleton instance of the radio driver
RH_RF24 rf24;

void setup() 
{
  Serial.begin(9600);
  if (!rf24.init())
    Serial.println("init failed");
}
void loop()
{
  Serial.println("Sending to rf24_server");
  // Send a message to rf24_server
  uint8_t data[] = "Nepal-PQ1-Communication Module Testing";
  rf24.send(data, sizeof(data));
  Serial.println(sizeof(data));
  
  rf24.waitPacketSent();
  // Now wait for a reply
  uint8_t buf[RH_RF24_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);

  if (rf24.waitAvailableTimeout(500))
  { 
    // Should be a reply message for us now   
    if (rf24.recv(buf, &len))
    {
      Serial.print("got reply: ");
      Serial.println((char*)buf);
    }
    else
    {
      Serial.println("recv failed");
    }
  }
  else
  {
    Serial.println("No reply, is rf24_server running?");
  }
  delay(400);
}
