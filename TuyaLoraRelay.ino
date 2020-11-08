
#include <SPI.h>
#include <LoRa.h>

#define SCK     15
#define MISO    14
#define MOSI    16
#define SS      8
#define RST     4
#define DI0     7
#define BAND    868E6  // 915E6
#define PABOOST true
#define RELAY_PIN 2
  
byte localAddress = 0x01;     // address of this device

void setup() {
  Serial.begin(9600);
  Serial.println("LoRa Receiver");
  LoRa.setPins(SS,RST,DI0);
  if (!LoRa.begin(BAND,PABOOST )) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }
  pinMode(RELAY_PIN, OUTPUT);
}

void loop() {
  onReceive(LoRa.parsePacket());
}

void onReceive(int packetSize) {
  if (packetSize == 0) return;

  int recipient = LoRa.read();
  byte sender = LoRa.read();
  uint8_t dataType = LoRa.read();
  uint8_t dataVal1 = LoRa.read();
  uint8_t dataVal2 = LoRa.read();
  uint16_t dataVal = (uint16_t)dataVal1 << 8 | dataVal2;

  String incoming = "";                 // payload of packet

  while (LoRa.available()) {            // can't use readString() in callback, so
    incoming += (char)LoRa.read();      // add bytes one by one
  }

  // if message is for this device, or broadcast, print details:
  Serial.println("Received from: 0x" + String(sender, HEX));
  Serial.println("Sent to: 0x" + String(recipient, HEX));
  Serial.println("dataType: " + String(dataType));
  Serial.println("dataVal: ");
  Serial.println(dataVal);
  Serial.println("RSSI: " + String(LoRa.packetRssi()));
  Serial.println("Snr: " + String(LoRa.packetSnr()));
  Serial.println();

  if (localAddress == recipient && dataVal == 1) {
      digitalWrite(RELAY_PIN, HIGH);
  }
  if (localAddress == recipient && dataVal == 0) {
      digitalWrite(RELAY_PIN, LOW);
  }
}
