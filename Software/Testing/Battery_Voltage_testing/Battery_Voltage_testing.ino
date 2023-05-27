#include <WiFi.h>
#include <WiFiUdp.h>

#define bat_pin 26
#define diode_vol_drop 0.7

#ifndef APSSID
#define APSSID "PicoW"
#define APPSW "password"
#endif
#define UDP_PKT_MAX_SIZE 16

unsigned int localPort = 8888;
char packetBuffer[UDP_PKT_MAX_SIZE + 1];
char ReplyBuffer[20]; // a string to send back

WiFiUDP Udp;  // Object for WiFi UDP class

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  
  pinMode(bat_pin, INPUT);
  Serial.begin(115200);
  
  WiFi.mode(WIFI_AP);
  WiFi.begin(APSSID, APPSW);
  while (WiFi.status() != WL_CONNECTED) {
        Serial.print(".");
        delay(500);
  }
  Udp.begin(localPort);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(1000);
  digitalWrite(LED_BUILTIN, LOW);
}

void loop() {
    int packetSize = Udp.parsePacket();
    if(packetSize) {
        float reading = analogRead(bat_pin) * 3.3 / 1023 * 1300 / 1000 + diode_vol_drop;    
        // put this reading into the buffer
        sprintf(ReplyBuffer, "%f", reading);
        // send the reading back to the sender
        Udp.beginPacket(Udp.remoteIP(), 8888);
        Udp.write(ReplyBuffer);
        Udp.endPacket();
        digitalWrite(LED_BUILTIN, HIGH);
    }
    
    delay(100);
    digitalWrite(LED_BUILTIN, LOW);
}