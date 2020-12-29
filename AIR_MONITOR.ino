#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <LiquidCrystal_I2C.h>
#include <pms.h>
#include "MHZ19.h"

#ifndef STASSID
#define STASSID "AIR"
#define STAPSK  "12131415"
#endif
#define interruptPin D5
#define SEALEVELPRESSURE_HPA (1013.25)

#define LOG_PERIOD 10000
const char* ssid = STASSID;
const char* password = STAPSK;

const int rx_pin = 13; //Serial rx pin no
const int tx_pin = 15; //Serial tx pin no

MHZ19 *mhz19_uart = new MHZ19(rx_pin,tx_pin);
LiquidCrystal_I2C lcd(0x27, 20, 4);
ESP8266WebServer server(80);
Adafruit_BME280 bme;
Pmsx003 pms(D1, D2);
const auto n = Pmsx003::Reserved;
Pmsx003::pmsData data[n];

measurement_t m;

unsigned long previousMillis;
unsigned long counts;
unsigned long radiation;
float ch20;
float temperature, humidity, pressure, altitude;

void handleRoot() {
  server.send(200, "text/html", SendHTML());
}

String SendHTML(){
  String ptr = "<!DOCTYPE html> <html>\n";
  ptr +="<head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0, user-scalable=no\">\n";
  ptr +="<title>ESP8266 Weather Station</title>\n";
  ptr +="<style>html { font-family: Helvetica; display: inline-block; margin: 0px auto; text-align: center;}\n";
  ptr +="body{margin-top: 50px;} h1 {color: #444444;margin: 50px auto 30px;}\n";
  ptr +="p {font-size: 24px;color: #444444;margin-bottom: 10px;}\n";
  ptr +="</style>\n";
  ptr +="</head>\n";
  ptr +="<body>\n";
  ptr +="<div id=\"webpage\">\n";
  ptr +="<h1>ESP8266 Weather Station</h1>\n";
  ptr +="<p>Temperature: ";
  ptr +=temperature;
  ptr +="&deg;C</p>";
  ptr +="<p>Humidity: ";
  ptr +=humidity;
  ptr +="%</p>";
  ptr +="<p>Pressure: ";
  ptr +=pressure;
  ptr +="hPa</p>";
  ptr +="<p>Altitude: ";
  ptr +=altitude;
  ptr +="m</p>";
  for (size_t i = Pmsx003::PM1dot0; i < n; ++i) { 
         ptr +="<p>";
         ptr +=data[i];
         ptr+="\t";
         ptr+=Pmsx003::dataNames[i];   
         ptr+=" [";  
         ptr+=Pmsx003::metrics[i];
         ptr+="]";
         ptr +="</p>";
      }
  ptr +="<p>";
  ptr +="CO2 level ";
  ptr +=m.co2_ppm;
  ptr +=" ppm";
  ptr +="</p>";
  ptr +="<p>";
  ptr +="CO2 temperature ";
  ptr +=m.temperature;
  ptr +="&deg;C</p>";
  ptr +="<p>";
  ptr +=6*radiation;
  ptr +=" puls/min";
  ptr +="</p>";
  ptr +="<p>";
  ptr +=ch20;
  ptr +=" CH20 ppm";
  ptr +="</p>";
  ptr +="</div>\n";
  ptr +="</body>\n";
  ptr +="</html>\n";
  return ptr;
}

void handleNotFound() {
  String message = "ERROR";
  server.send(404, "text/plain", message);
}

void IRAM_ATTR tube_impulse(){       //Функция подсчета имульсов
  counts++;
}

void measure(){                              
  unsigned long currentMillis = millis();
  if(currentMillis - previousMillis > LOG_PERIOD){
    previousMillis = currentMillis;
    temperature = bme.readTemperature();
    humidity = bme.readHumidity();
    pressure = bme.readPressure() / 100.0F;
    altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);
    radiation=counts;
    Pmsx003::PmsStatus status = pms.read(data, n);
    m = mhz19_uart->getMeasurement();
    counts = 0;
    ch20=(analogRead(A0)-124)/99.2;
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("T=");
    lcd.print(int(temperature));
    lcd.print(" H=");
    lcd.print(int(humidity));
    lcd.print(" P=");
    lcd.print(int(pressure));
    lcd.setCursor(0, 1);
    lcd.print("C=");
    lcd.print(int(m.co2_ppm));
    
    lcd.print(" R=");
    lcd.print(int(6*radiation));
    lcd.print(" CH2=");
    lcd.print(ch20);
    lcd.setCursor(0, 2);
    lcd.print("PM1="); 
    lcd.print(data[Pmsx003::PM1dot0]);
    lcd.print(" PM2.5="); 
    lcd.print(data[Pmsx003::PM1dot0+1]);
    lcd.setCursor(0, 3);
    lcd.print("PM10="); 
    lcd.print(data[Pmsx003::PM1dot0+2]);
  //  lcd.print(" ");
    //
  }
}
void setup(void) {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.println("");
 // interrupts();
  pinMode(interruptPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(interruptPin), tube_impulse, RISING);
  lcd.begin(20,4);
  lcd.init();
  lcd.backlight();
  bme.begin(0x76); 
  pms.begin();
  pms.waitForData(Pmsx003::wakeupTime);
  pms.write(Pmsx003::cmdModeActive);  
  mhz19_uart->begin(rx_pin, tx_pin);
  mhz19_uart->setAutoCalibration(false);
  // Wait for connection
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.print("Connected to ");
  Serial.println(ssid);
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(WiFi.localIP());

  if (MDNS.begin("esp8266")) {
    Serial.println("MDNS responder started");
  }

  server.on("/", handleRoot);
  server.onNotFound(handleNotFound);

  server.begin();
  Serial.println("HTTP server started");
}

void loop(void) {
  server.handleClient();
  MDNS.update();
  measure();
}
