#include <SPI.h>
#include <Ethernet.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <EthernetUdp.h>
#include <Preferences.h>
#include <Dmx_ESP32.h>


// CONFIG
#define DMX_TX_PIN 3
#define DMX_TX_PIN2 17
#define TX_ENABLE -1 
#define UNIVERSE_SIZE 512
#define DMX_BAUD 250000
#define ARTNET_PORT 6454
#define ETH_CS 15
#define DMX_PORT &Serial2

dmxTx dmxSend(DMX_PORT, DMX_TX_PIN2, TX_ENABLE, 0, LOW);


Preferences prefs;

IPAddress local_IP(10, 10, 1, 100);
IPAddress fallback_IP(10, 10, 1, 1);
IPAddress gateway(10, 10, 1, 1);
IPAddress subnet(255, 255, 255, 0);

EthernetServer ethServer(80);
WiFiServer wifiServer(80);
EthernetUDP ethUdp;
WiFiUDP wifiUdp;

bool useEthernet = false;
uint8_t dmxData[UNIVERSE_SIZE + 1] = {0};
uint16_t targetUniverse = 0;


// DMX
void sendDMXFrame(uint8_t* data, int count) {
  
      dmxSend.transmit();
      for (int i = 1; i < count; i++) {
        dmxSend.write(data[i], i);
      }
      
}

// Web interface
void handleWebServer() {
  EthernetClient ec = ethServer.available();
  WiFiClient wc = wifiServer.available();
  Client* client = useEthernet ? (Client*)&ec : (Client*)&wc;

  if (client && client->connected()) {
    String req = "";
    unsigned long timeout = millis();
    while (client->connected() && millis() - timeout < 1000) {
      if (client->available()) {
        char c = client->read();
        req += c;
        if (req.endsWith("\r\n\r\n")) break;
      }
    }

    if (req.indexOf("POST /save") >= 0 || req.indexOf("GET /?universe=") >= 0) {
      int start = req.indexOf("universe=") + 9;
      int end = req.indexOf("&", start);
      if (end == -1) end = req.indexOf(" ", start);
      String val = req.substring(start, end);
      targetUniverse = val.toInt();
      prefs.begin("config", false);
      prefs.putUInt("universe", targetUniverse);
      prefs.end();
      client->println("HTTP/1.1 200 OK");
      client->println("Content-Type: text/html\n");
      client->println("<html><body>Instellingen opgeslagen. Herstart...</body></html>");
      client->stop();
      delay(1000);
      ESP.restart();
    }

    // HTML output
    client->println("HTTP/1.1 200 OK");
    client->println("Content-Type: text/html\n");
    client->println("<!DOCTYPE html><html><head><title>DMX Config</title></head><body>");
    client->println("<h2>ESP DMX Configuratie</h2>");
    client->print("<form action=\"/\" method=\"get\">");
    client->printf("Universe: <input name=\"universe\" value=\"%d\"><br>", targetUniverse);
    client->println("<input type=\"submit\" value=\"Opslaan & Herstart\"></form>");
    client->println("</body></html>");
    client->stop();
  }
}

// Artnet
void handleArtnetPacket() {
  int packetSize = useEthernet ? ethUdp.parsePacket() : wifiUdp.parsePacket();
  if (packetSize > 0) {
    uint8_t buffer[530];
    int len = useEthernet ? ethUdp.read(buffer, 530) : wifiUdp.read(buffer, 530);
    if (len < 18 || memcmp(buffer, "Art-Net", 7) != 0) return;
      // print first 10 bytes raw to confirm Art-Net header
    Serial.print("Raw bytes: ");
    for(int i = 0; i < 10 && i < packetSize; i++) {
      Serial.printf("%02X ", buffer[i]);
    }
    Serial.println();

    uint16_t opcode = buffer[8] | (buffer[9] << 8);
    if (opcode != 0x5000) return;

    uint16_t universe = buffer[14] | (buffer[15] << 8);
    if (universe != targetUniverse) return;

    uint16_t length = buffer[17] | (buffer[16] << 8);
    if (length > UNIVERSE_SIZE) length = UNIVERSE_SIZE;

    Serial.printf("ch1=%d ch2=%d ch3=%d ch4=%d\n", 
          buffer[18], buffer[19], buffer[20], buffer[21]);
    dmxData[0] = 0;
    memcpy(&dmxData[1], &buffer[18], length);
    Serial.println("sendingDATA");
    if (dmxSend.readyToTransmit()) { 
      sendDMXFrame(dmxData, UNIVERSE_SIZE + 1);
    }
  }
}

void setup() {
  Serial.begin(115200);
  if (!dmxSend.configure()) {
    Serial.println("DMX Tx was previously configured.");
  }
  // Load universe
  prefs.begin("config", true);
  targetUniverse = prefs.getUInt("universe", 0);
  prefs.end();

  // Init Ethernet
  pinMode(ETH_CS, OUTPUT);
  digitalWrite(ETH_CS, HIGH);
  SPI.begin();
  Ethernet.init(ETH_CS);
  Ethernet.begin((uint8_t[]){0xDE,0xAD,0xBE,0xEF,0xFE,0xED}, local_IP);
  delay(300);

  if (Ethernet.hardwareStatus() == EthernetW5500 && Ethernet.linkStatus() == LinkON) {
    ethServer.begin();
    ethUdp.begin(ARTNET_PORT);
    useEthernet = true;
    Serial.print("Ethernet actief op ");
    Serial.println(Ethernet.localIP());
  } else {
    Serial.println("Ethernet faalt. Start WiFi fallback.");
    WiFi.disconnect(true);
    delay(200);
    WiFi.mode(WIFI_OFF);
    delay(200);
    WiFi.mode(WIFI_AP);
    WiFi.softAPConfig(fallback_IP, fallback_IP, subnet);
    WiFi.softAP("ESP-DMX", "dmxPassword");
    wifiServer.begin();
    wifiUdp.begin(ARTNET_PORT);
    useEthernet = false;
    Serial.print("WiFi actief op ");
    Serial.println(WiFi.softAPIP());
  }

  Serial.println("Klaar voor Artnet.");
}

void loop() {
  handleArtnetPacket();
 // handleWebServer();
}
