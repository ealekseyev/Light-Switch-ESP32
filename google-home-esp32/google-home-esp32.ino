/*
 * Reply codes: from ESP8266:
 * ! @ # $ % ^ & *
 * ! - temperature, humidity, pressure. Ex: "!22.3;76.4;101.3" Format: "!temp; hum; press"
 * @ - panic, ip other than server has sent something to esp. Ex: "@192.168.1.137" Format: "@IPADDRESS
 */
#include <AdafruitIO_WiFi.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Servo.h>

#define IO_USERNAME  "evan_not_devin"
#define IO_KEY       "aio_MtpQ88nhTAV5dXXu2UBFdIpNijcA"

const char* WIFI_SSID = "Despacito-1";
const char* WIFI_PASS = "kauai2018";

AdafruitIO_WiFi io(IO_USERNAME, IO_KEY, WIFI_SSID, WIFI_PASS);
AdafruitIO_Feed *esp32control = io.feed("LED");

WiFiUDP udp;
Servo servo;

Adafruit_BME280 bme;
const uint8_t bmeAddr = 0x76;

// pi server's IP address
IPAddress piIP(192, 168, 1, 150);
const char* ESPhostname = "esp8266";
const unsigned int UDPPort = 61205;
char* replyBuffer = "XX.Y;ZZ.Q;AAA.B";

// static ip config
IPAddress localIP(192, 168, 1, 162);
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 255, 0);
IPAddress primaryDNS(1, 1, 1, 1);

int offPos = 140;
int neutralPos = 100;
int onPos = 50;

void setup() {
  Serial.begin(115200);

  // connect to the sensor
  bme.begin(bmeAddr);

  // config static IP
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  WiFi.config(localIP, gateway, subnet, primaryDNS);
  if(WiFi.status() == WL_CONNECTED) {
    Serial.println("Connected");
  }
  else {
    Serial.println("not connected");
  }

  // connect to Adafruit IO
  io.connect();
  esp32control->onMessage(onButtonPressed);

  // start UDP on given port
  udp.begin(UDPPort);

  // start MDNS server
  //MDNS.begin(ESPhostname, localIP);
  //MDNS.addService("http", "tcp", 80);

  servo.attach(12);
  delay(100);
  servo.write(100);
  delay(300);
  digitalWrite(12, LOW);
}

void loop() {
  // stay connected to adafruit.io
  io.run();
  // keep running MDNS server
  //MDNS.update();
  // check for the time, every 20 s send UDP
  checkForTime();
}

void getUdp() {
  char packetBuffer[UDP_TX_PACKET_MAX_SIZE + 1];
  int packetSize = udp.parsePacket();
  
  if(packetSize) {
    char* ip = (char*)udp.remoteIP().toString().c_str();
    int n = udp.read(packetBuffer, UDP_TX_PACKET_MAX_SIZE);
    packetBuffer[n] = 0;
    
    if(ip == "192.168.1.150") {
      
    } else {
      replyBuffer = strcat("@", ip);
      udp.beginPacket(piIP, UDPPort);
      udp.write(replyBuffer);
      udp.endPacket();
    }
  }
}

void checkForTime() {
  static unsigned long pastTime = 0;
  if(millis() - pastTime >= 20000) {
    Serial.println("sending");
    float temp_c = float(round(bme.readTemperature()*10)/10.0);
    float hum_c = float(round(bme.readHumidity()*10)/10.0);
    float press_c = float(round(bme.readPressure()/10)/10.0);
    
    String buffer = "!" + (String) temp_c;
    buffer += ";" + (String)hum_c;
    buffer += ";" + (String) press_c;
    buffer.toCharArray(replyBuffer, buffer.length());
    Serial.println(buffer);
    Serial.println(replyBuffer);
    
    udp.beginPacket(piIP, UDPPort);
    udp.write(replyBuffer);
    udp.endPacket();
    pastTime = millis();
  }
}

void onButtonPressed(AdafruitIO_Data *data) {
  String dataVal = data->value();
  Serial.println("Got data from onButtonPressed: " + dataVal);
  
  if(dataVal == "ON") {
      turnLightsOn();
  }
  
  else if(dataVal == "OFF") {
    turnLightsOff();
  } 
  
  else if(dataVal == "RESTART") {
    ESP.restart();
  }
}

void turnLightsOn() {
  servo.write(neutralPos);
  delay(100);
  for(int i = neutralPos; i > onPos; i-=10) {
    servo.write(i);
    delay(30);
  }
  for(int i = onPos; i < neutralPos; i+=5) {
    servo.write(i);
    delay(30);
  }
  delay(300);
  digitalWrite(12, LOW);
}

void turnLightsOff() {
  servo.write(neutralPos);
  delay(100);
  for(int i = neutralPos; i < offPos; i+=10) {
    servo.write(i);
    delay(30);
  }
  for(int i = onPos; i > neutralPos; i-=5) {
    servo.write(i);
    delay(30);
  }
  delay(300);
  digitalWrite(12, LOW);
}
