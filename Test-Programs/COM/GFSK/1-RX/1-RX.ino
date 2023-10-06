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
  if (rf24.available())
  {
    // Should be a message for us now
    uint8_t buf[RH_RF24_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);
    if (rf24.recv(buf, &len))
    {
      Serial.print("got request: ");
      Serial.println((char*)buf);
      Serial.print("RSSI: ");
      Serial.println((uint8_t)rf24.lastRssi(), DEC);

      // Send a reply
      uint8_t data[] = "And hello back to you";
      rf24.send(data, sizeof(data));
      rf24.waitPacketSent();
      Serial.println("Sent a reply");
    }
    else
    {
      Serial.println("recv failed");
    }
  }
}
