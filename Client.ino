#include <RH_RF95.h> // RFM9x radio transmitter/reciever library
#include <SPI.h>

#define RFM95_INT 9 // "A"  //***Need to update pin-assignments if they don't match arduino/break-out board
#define RFM95_CS 10 // "B"
#define RFM95_RST 11 // "C"
#define RF95_FREQ 433 // match to server freq

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

// Blinks upon receipt
#define LED 13 //***Need to review this pin assignment

//Below is taken from datasheet example, maybe junk
/* for shield
#define RFM95_CS 10
#define RFM95_RST 9
#define RFM95_INT 7
*/

void setup() {
  //1. INITIALIZE THE RFM95 RADIO TRANSMITTER
  pinMode(LED,OUTPUT);
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);
  Serial.begin(1000000); //Match to server baud rate
  while (!Serial) {
    delay(1);
  }
  delay(100); 
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);
  while (!rf95.init()) {
    Serial.println("Radio init failed");
    Serial.println("Uncomment '#define SERIAL_DEBUG' in RH_RF95.cpp for detailed debug info");
    while (1);
  }
  Serial.println("Radio init successful"); 
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
    while (1);
  }
  Serial.print("Set Freq to: "); Serial.println(RF95_FREQ);
  rf95.setTxPower(23, false); //Match to server TxPower

}

void loop() {
  if (rf95.available()){ // The data packet has likely arrived!
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN]; // [this and subsequent code definitely needs review and debugging]
    uint8_t len = sizeof(buf);
    if (rf95.recv(buf, &len)){
      digitalWrite(LED, HIGH);
      float x = *((float *)buf);
      Serial.println((char*)buf); 
      delay(10);
      digitalWrite(LED, LOW);
    }
    else{
      Serial.println("Receive failed");
    }
  }
}
