#include <SPI.h>
#include <ETH.h>
#include <NetworkUdp.h>
#include <Dmx_ESP32.h>

// W5500 SPI pins — ESP32 DevKit VSPI
#define ETH_PHY_TYPE  ETH_PHY_W5500
#define ETH_PHY_ADDR  1
#define ETH_PHY_CS    5    // SCS  → GPIO 5
#define ETH_PHY_IRQ   -1
#define ETH_PHY_RST   -1   // tie RST to 3.3V
#define ETH_SPI_SCK   18   // SCLK → GPIO 18
#define ETH_SPI_MISO  19   // MISO → GPIO 19
#define ETH_SPI_MOSI  23   // MOSI → GPIO 23

// DMX output
#define DMX_TX_PIN      17
#define TX_ENABLE       -1
#define DMX_PORT        &Serial2
#define UNIVERSE_SIZE   512

// ArtNet
#define ARTNET_PORT     6454
#define TARGET_UNIVERSE 0

// Static IP — set Network Address to this in TouchDesigner
IPAddress local_IP(169, 254, 212, 50);
IPAddress gateway(169, 254, 212, 200);
IPAddress subnet(255, 255, 0, 0);

NetworkUDP udp;
dmxTx dmxSend(DMX_PORT, DMX_TX_PIN, TX_ENABLE, 0, LOW);
uint8_t dmxData[UNIVERSE_SIZE + 1] = {0};
bool ethReady = false;


void sendDMXFrame(uint8_t* data, int count) {
  dmxSend.transmit();
  for (int i = 1; i < count; i++) {
    dmxSend.write(data[i], i);
  }
}


void handleArtNet() {
  int packetSize = udp.parsePacket();
  if (packetSize <= 0) return;

  uint8_t buf[530];
  int len = udp.read(buf, sizeof(buf));

  if (len < 18)                         return;
  if (memcmp(buf, "Art-Net\0", 8) != 0) return;

  uint16_t opcode = buf[8] | (buf[9] << 8);
  if (opcode != 0x5000) return;

  uint16_t universe = buf[14] | (buf[15] << 8);
  if (universe != TARGET_UNIVERSE) return;

  uint16_t length = (buf[16] << 8) | buf[17];
  if (length > UNIVERSE_SIZE) length = UNIVERSE_SIZE;

  dmxData[0] = 0x00;
  memcpy(&dmxData[1], &buf[18], length);

  if (dmxSend.readyToTransmit()) {
    sendDMXFrame(dmxData, length + 1);
  }
}


void onEthEvent(arduino_event_id_t event, arduino_event_info_t info) {
  switch (event) {
    case ARDUINO_EVENT_ETH_START:
      Serial.println("ETH: started");
      ETH.config(local_IP, gateway, subnet);
      break;
    case ARDUINO_EVENT_ETH_CONNECTED:
      Serial.println("ETH: cable connected");
      break;
    case ARDUINO_EVENT_ETH_GOT_IP:
      Serial.print("ETH: IP = ");
      Serial.println(ETH.localIP());
      udp.begin(ARTNET_PORT);
      ethReady = true;
      break;
    case ARDUINO_EVENT_ETH_DISCONNECTED:
      Serial.println("ETH: disconnected");
      ethReady = false;
      break;
    default:
      break;
  }
}


void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n--- Boot ---");

  if (!dmxSend.configure()) {}

  Network.onEvent(onEthEvent);
  SPI.begin(ETH_SPI_SCK, ETH_SPI_MISO, ETH_SPI_MOSI, ETH_PHY_CS);
  ETH.begin(ETH_PHY_TYPE, ETH_PHY_ADDR, ETH_PHY_CS, ETH_PHY_IRQ, ETH_PHY_RST, SPI);
}


void loop() {
  if (ethReady) {
    handleArtNet();
  }
}
