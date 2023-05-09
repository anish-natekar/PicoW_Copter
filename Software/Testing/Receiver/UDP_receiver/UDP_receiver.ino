#include <WiFi.h>
#include <WiFiUdp.h>

#ifndef APSSID
#define APSSID "PicoW"    // name of your PicoW Hotspot
#define APPSW "password" // password of your PicoW Hotspot
#endif
#define UDP_PKT_MAX_SIZE 16 //  number of characters in a UDP packet

unsigned int localPort = 8888;  // local port for UDP communication
char packetBuffer[UDP_PKT_MAX_SIZE + 1];  // max number of characters received in one message
int Throttle, Roll, Pitch, Yaw; // values received from each channel
int prev;

WiFiUDP Udp; // Object for WIFI UDP class

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_AP); // Access Point mode
  WiFi.begin(APSSID, APPSW);  // By default static IP for PicoW will be 192.168.42.1
  while(WiFi.status() != WL_CONNECTED) {
    Serial.print('.');  // waiting for connection
    delay(500); // 0.5 sec delay
  }    
  Serial.print("\nConnected! IP address: ");
  Serial.println(WiFi.localIP());   // The IP Address is 192.168.42.1
  Serial.printf("UDP server on port %d\n", localPort);  // Port is 8888
  Udp.begin(localPort); // start listening on port 8888
}

void loop() {
  // if there is data available to read then read a packet
  int packetSize = Udp.parsePacket();
  if(packetSize) {  // if packet size is > 0
    prev = micros();
    int n = Udp.read(packetBuffer, UDP_PKT_MAX_SIZE); // read the data from UDP packet into packetBuffer
    packetBuffer[n] = '\0'; // character for end of string
    char ch1[5], ch2[5], ch3[5], ch4[5];  // 
    ch1[4] = '\0'; ch2[4] = '\0'; ch3[4] = '\0'; ch4[4] = '\0';
    for(int i=0; i<4; i++) {
      // Spliting the packets into four values of 4 characters each
      ch1[i] = packetBuffer[i];
      ch2[i] = packetBuffer[i+4];
      ch3[i] = packetBuffer[i+8];
      ch4[i] = packetBuffer[i+12];            
    }    
    // converting string/character arrays to integer
    Yaw = atoi(ch1);
    Throttle = atoi(ch2);
    Roll = atoi(ch3);
    Pitch = atoi(ch4);    
    Serial.printf("Yaw = %d, Throttle = %d, Roll = %d, Pitch = %d\n", Yaw, Throttle, Roll, Pitch);
    Serial.printf("Time taken = %d\n", micros() - prev);
  }  
}
