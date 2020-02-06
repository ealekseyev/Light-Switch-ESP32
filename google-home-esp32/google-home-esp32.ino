/*
 * Reply codes: from ESP8266:
 * ! @ # $ % ^ & *
 * ! - temperature, humidity, pressure. Ex: "!22.3; 76.4; 101.3" Format: "!temp; hum; press"
 * @ - panic, ip other than server has sent something to esp. Ex: "@192.168.1.137" Format: "@IPADDRESS
 * 
 * 
 */
#include <AdafruitIO_WiFi.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
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
char* replyBuffer = "null";

//static ip config
IPAddress localIP(192, 168, 1, 162);
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 255, 0);
IPAddress primaryDNS(1, 1, 1, 1);

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
  MDNS.begin(ESPhostname, localIP);
  MDNS.addService("http", "tcp", 80);

  // start OTA programming
  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";
    } else { // U_FS
      type = "filesystem";
    }
  });
  ArduinoOTA.begin();

  servo.attach(12);
  servo.write(90);
  delay(300);
  servo.write(100);
  delay(300);
  servo.write(95);
  delay(300);
  digitalWrite(12, LOW);
}

void loop() {
  // stay connected to adafruit.io
  // keep running MDNS and OTA server
  io.run();
  MDNS.update();
  io.run();
  ArduinoOTA.handle();
  io.run();

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
    float press_c = float(round(bme.readPressure()*10)/10.0);
    
    String buffer = "!" + (String) temp_c;
    buffer += "; " + (String)hum_c;
    buffer += "; " + (String) press_c;
    buffer.toCharArray(replyBuffer, buffer.length());
    
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
    servo.write(95);
    delay(200);
    for(int i = 92; i > 54; i--) {
      servo.write(i);
      delay(3);
    }
    for(int i= 55; i < 95; i++) {
      servo.write(i);
      delay(3);
    }
    servo.write(95);
    delay(200);
    digitalWrite(12, LOW);
  }
  
  
  else if(dataVal == "OFF") {
    servo.write(95);
    delay(200);
    for(int i = 92; i < 141; i++) {
      servo.write(i);
      delay(3);
    }
    for(int i = 140; i > 95; i--) {
      servo.write(i);
      delay(3);
    }
    servo.write(95);
    delay(200);
    digitalWrite(12, LOW);
  }

  
  else if(dataVal == "RESTART") {
    ESP.restart();
  }

  
  else if(dataVal == "BONANZA") {
    servo.write(95);
    delay(200);
    for(int i = 0; i < 5; i++) {
      for(int i = 92; i < 151; i++) {
        servo.write(i);
        delay(2);
      }
      for(int i = 150; i > 95; i--) {
        servo.write(i);
        delay(2);
      }
      for(int i = 92; i > 44; i--) {
        servo.write(i);
        delay(2);
      }
      for(int i= 45; i < 95; i++) {
        servo.write(i);
        delay(2);
      }
    }
    servo.write(95);
    delay(200);
    digitalWrite(12, LOW);
  }
}
