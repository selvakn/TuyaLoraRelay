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

byte DP = 0x01;     // address of this device

struct TUYA {
  uint8_t cmd_status = 0;                 // Current status of serial-read
  uint8_t cmd_checksum = 0;               // Checksum of tuya command
  uint16_t data_len = 0;                   // Data lenght of command
  char *buffer = nullptr;                 // Serial receive buffer
  int byte_counter = 0;                   // Index in serial receive buffer
} Tuya;

#define TUYA_CMD_HEARTBEAT     0x00
#define TUYA_CMD_QUERY_PRODUCT 0x01
#define TUYA_CMD_MCU_CONF      0x02
#define TUYA_CMD_WIFI_STATE    0x03
#define TUYA_CMD_WIFI_RESET    0x04
#define TUYA_CMD_WIFI_SELECT   0x05
#define TUYA_CMD_SET_DP        0x06
#define TUYA_CMD_STATE         0x07
#define TUYA_CMD_QUERY_STATE   0x08
#define TUYA_CMD_SET_TIME      0x1C

#define TUYA_TYPE_BOOL         0x01
#define TUYA_TYPE_VALUE        0x02
#define TUYA_TYPE_STRING       0x03
#define TUYA_TYPE_ENUM         0x04

#define TUYA_BUFFER_SIZE       256

unsigned long lastSendTime = 0;                    // last send time
unsigned int statusUpdateInterval = 5000;          // interval between sends
bool poweredOn = false;

void setup() {
  Serial.begin(9600);
  Serial.println("LoRa Receiver");

  Tuya.buffer = (char*)(malloc(TUYA_BUFFER_SIZE));

  LoRa.setPins(SS, RST, DI0);
  if (!LoRa.begin(BAND, PABOOST)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }
  pinMode(RELAY_PIN, OUTPUT);
}

void loop() {
  onReceive(LoRa.parsePacket());

  if (millis() - lastSendTime > statusUpdateInterval) {
    sendStatusUpdate();
    lastSendTime = millis();
    statusUpdateInterval = statusUpdateInterval + random(1000) - random(1000);
  }
}

void TuyaSendResponseOverLora(uint8_t cmd, uint8_t payload[] = nullptr, uint16_t payload_len = 0)
{
  uint8_t checksum = (0xFF + cmd + (payload_len >> 8) + (payload_len & 0xFF));

  LoRa.beginPacket();
  LoRa.write(0x55);                  // Tuya header 55AA
  LoRa.write(0xAA);
  LoRa.write((uint8_t)0x00);         // version 00
  LoRa.write(cmd);                   // Tuya command
  LoRa.write(payload_len >> 8);      // following data length (Hi)
  LoRa.write(payload_len & 0xFF);    // following data length (Lo)
  for (uint32_t i = 0; i < payload_len; ++i) {
    LoRa.write(payload[i]);
    checksum += payload[i];
  }
  LoRa.write(checksum);
  LoRa.endPacket();
}

void sendStatusUpdate() {
  uint8_t statusByte = poweredOn ? 0x01 : 0x00;
  uint8_t payloadBuffer[] = {0x01, 0x01, 0x00, 0x01, statusByte};
  Serial.println("sending status update over lora");
  TuyaSendResponseOverLora(TUYA_CMD_STATE, payloadBuffer, sizeof(payloadBuffer));
}

void onReceive(int packetSize) {
  if (packetSize == 0) return;

  Serial.println("RSSI: " + String(LoRa.packetRssi()));
  Serial.println("Snr: " + String(LoRa.packetSnr()));
  Serial.println("Packet Size: " + String(packetSize));

  while (LoRa.available()) {
    ProcessByte(LoRa.read());
  }
}

char* ToHex_P(unsigned char * in, size_t insz, char * out, size_t outsz)
{
  unsigned char * pin = in;
  const char * hex = "0123456789ABCDEF";
  char * pout = out;
  for (; pin < in + insz; pout += 3, pin++) {
    pout[0] = hex[(*pin >> 4) & 0xF];
    pout[1] = hex[ *pin     & 0xF];
    pout[2] = ':';
    if (pout + 3 - out > outsz) {
      /* Better to truncate output string than overflow buffer */
      /* it would be still better to either return a status */
      /* or ensure the target buffer is large enough and it never happen */
      break;
    }
  }
  pout[-1] = 0;
  return out;
}

unsigned int setCommandPacketLength(unsigned int dataLen) {
  return 6 + dataLen + 1; // Preamble(2) + version(1) + command byte[6] (1) + lengths'lengh (2) + length(n) + checksum(1)
}

void ProcessByte(uint8_t serial_in_byte)
{
  if (serial_in_byte == 0x55) {            // Start TUYA Packet
    Tuya.cmd_status = 1;
    Tuya.buffer[Tuya.byte_counter++] = serial_in_byte;
    Tuya.cmd_checksum += serial_in_byte;
    //    Serial.println("0x55");
  }
  else if (Tuya.cmd_status == 1 && serial_in_byte == 0xAA) { // Only packtes with header 0x55AA are valid
    Tuya.cmd_status = 2;

    Tuya.byte_counter = 0;
    Tuya.buffer[Tuya.byte_counter++] = 0x55;
    Tuya.buffer[Tuya.byte_counter++] = 0xAA;
    Tuya.cmd_checksum = 0xFF;
    //    Serial.println("cmd_status: 2");
  }
  else if (Tuya.cmd_status == 2) {
    if (Tuya.byte_counter == 5) { // Get length of data
      Tuya.cmd_status = 3;
      Tuya.data_len = serial_in_byte;
      //      Serial.println("cmd_status: 3, found length: " + String(Tuya.data_len));
    }

    Tuya.cmd_checksum += serial_in_byte;
    Tuya.buffer[Tuya.byte_counter++] = serial_in_byte;
  }
  else if ((Tuya.cmd_status == 3) && (Tuya.byte_counter == (6 + Tuya.data_len)) && (Tuya.cmd_checksum == serial_in_byte)) { // Compare checksum and process packet
    //      Serial.println("checksum statisfied");

    Tuya.buffer[Tuya.byte_counter++] = serial_in_byte;

    uint16_t DataVal = 0;
    uint8_t dpId = 0;
    uint8_t dpDataType = 0;
    char DataStr[13];

    //      char hex_char[setCommandPacketLength(Tuya.data_len)*3];
    //      Serial.println(ToHex_P((unsigned char*)Tuya.buffer, setCommandPacketLength(Tuya.data_len), hex_char, sizeof(hex_char)));

    if (TUYA_CMD_HEARTBEAT == Tuya.buffer[3]) {
      uint8_t payload_buffer[] = {0x01};
      Serial.println("received heartbeat?");
    }

    if (TUYA_CMD_SET_DP == Tuya.buffer[3]) {
      //55 AA 03 07 00 0D 01 04 00 01 02 02 02 00 04 00 00 00 1A 40
      // 0  1  2  3  4  5  6  7  8  9 10 11 12 13 14 15 16 17 18 19

      uint8_t dpidStart = 6;
      snprintf_P(DataStr, sizeof(DataStr), PSTR("000000000000"));
      while (dpidStart + 4 < Tuya.byte_counter) {
        dpId = Tuya.buffer[dpidStart];
        dpDataType = Tuya.buffer[dpidStart + 1];
        uint16_t dpDataLen = Tuya.buffer[dpidStart + 2] << 8 | Tuya.buffer[dpidStart + 3];
        const unsigned char *dpData = (unsigned char*)&Tuya.buffer[dpidStart + 4];

        if (TUYA_TYPE_BOOL == dpDataType && dpDataLen == 1) {
          DataVal = dpData[0];
        }
        else if (TUYA_TYPE_VALUE == dpDataType && dpDataLen == 4) {
          uint32_t dpValue = (uint32_t)dpData[0] << 24 | (uint32_t)dpData[1] << 16 | (uint32_t)dpData[2] << 8 | (uint32_t)dpData[3] << 0;
          DataVal = dpValue;
        } else if (TUYA_TYPE_STRING == dpDataType) {
        } else if (TUYA_TYPE_ENUM == dpDataType && dpDataLen == 1) {
          DataVal = dpData[0];
        }

        dpidStart += dpDataLen + 4;
      }

      Serial.print("dpId: ");
      Serial.println(dpId, HEX);

      Serial.print("dpDataType: ");
      Serial.println(dpDataType, HEX);

      Serial.print("DataVal: ");
      Serial.println(DataVal);

      if (dpId == DP) {
        poweredOn = DataVal == 1;
        digitalWrite(RELAY_PIN, poweredOn ? HIGH : LOW);
      }

    }

    Tuya.byte_counter = 0;
    Tuya.cmd_status = 0;
    Tuya.cmd_checksum = 0;
    Tuya.data_len = 0;
  }                                                    // read additional packets from TUYA
  else if (Tuya.byte_counter < TUYA_BUFFER_SIZE - 1) { // add char to string if it still fits
    //      Serial.println("Tuya.cmd_status" + String(Tuya.cmd_status) + " Tuya.byte_counter: " + String(Tuya.byte_counter) + " Tuya.data_len: " + String(Tuya.data_len) + " serial_in_byte:" + serial_in_byte);
    //      Serial.println("reading more buffer");
    Tuya.buffer[Tuya.byte_counter++] = serial_in_byte;
    Tuya.cmd_checksum += serial_in_byte;
  } else {
    //      Serial.println("resetting");
    Tuya.byte_counter = 0;
    Tuya.cmd_status = 0;
    Tuya.cmd_checksum = 0;
    Tuya.data_len = 0;
  }
}
