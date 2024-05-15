#include <SPI.h>
#include <RH_RF95.h>

#define RFM95_CS    10  // Pin definitions
#define RFM95_INT   3
#define RFM95_RST   9
#define RF95_FREQ 908.0 // Default is 434.0 MHz
RH_RF95 rf95(RFM95_CS, RFM95_INT);
#define drone_power A1
#define lower_legs A2
#define death A3
#define shutoff A4
void setup()
{
  pinMode(drone_power, INPUT_PULLUP);
  pinMode(lower_legs, INPUT_PULLUP);
  pinMode(death, INPUT_PULLUP);
  pinMode(shutoff, INPUT_PULLUP);
  Serial.begin(9600);
  while (!Serial) ; // Wait for serial port to be available
  if (!rf95.init())
    Serial.println("init failed");
 
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
    while (1);
  }
  rf95.setSpreadingFactor(9);
  rf95.setSignalBandwidth(62500);
  rf95.setCodingRate4(8);
  rf95.setTxPower(23, false);
}
void loop()
{
  Serial.println(digitalRead(drone_power));
  //Serial.println(digitalRead(lower_legs));
  //Serial.println(digitalRead(death));

//   //Serial.println("Sending to rf95_server");
//   // Send a message to rf95_server
  if(digitalRead(shutoff) == LOW){
    uint8_t data[] = "drone power";
    rf95.send(data, sizeof(data));
    rf95.waitPacketSent();
  }
  delay(200);
  if(digitalRead(drone_power) == LOW){
    uint8_t data[] = "drone power";
    rf95.send(data, sizeof(data));
    rf95.waitPacketSent();
  }
  delay(200);
  if(digitalRead(lower_legs) == LOW){
    uint8_t data[] = "lower legs";
    rf95.send(data, sizeof(data));
    rf95.waitPacketSent();
  }
 delay(200);
  if(digitalRead(death) == LOW){
    uint8_t data[] = "release";
    rf95.send(data, sizeof(data));
    rf95.waitPacketSent();
  }
  delay(200);
//   // Now wait for a reply
//   uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
//   uint8_t len = sizeof(buf);
//   if (rf95.waitAvailableTimeout(3000))
//   {
//     // Should be a reply message for us now  
//     if (rf95.recv(buf, &len))
//    {
//       Serial.print("got reply: ");
//       Serial.println((char*)buf);
// //      Serial.print("RSSI: ");
// //      Serial.println(rf95.lastRssi(), DEC);    
//     }
//     else
//     {
//       Serial.println("recv failed");
//     }
//   }
//   else
//   {
//     Serial.println("No reply, is rf95_server running?");
//   }
//   //delay(400);
}