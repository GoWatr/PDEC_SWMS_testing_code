/*
 * Combined Code - All 13 Flowmeters + TDS + Hydrostatic OHT1 (Modbus) + OHT2 (4-20mA Analog)
 */

#include <Arduino.h>
#include <SoftwareSerial.h>
#include <WiFi.h>
#include <HTTPClient.h>

const char* ssid = "AaishtKhanna Shivani";
const char* password = "a1b1c1d1e1";

/*
const char* ssid = "test";
const char* password = "12345678";
*/

#define NUM_FLOWMETERS 13

const char* flowmeter_id[NUM_FLOWMETERS] = {
  "GWFMHIR01","GWFMHIR02","GWFMHIR03","GWFMHIR04","GWFMHIR05",
  "GWFMHIR06","GWFMHIR07","GWFMHIR08","GWFMHIR09","GWFMHIR10",
  "GWFMHIR11","GWFMHIR12","GWFMHIR13"
};

int64_t totalizer[NUM_FLOWMETERS];
int flowrate[NUM_FLOWMETERS];

float tdsValue = 0.0;
uint16_t hydRawValue = 0;
float oht1Percent = 0.0;

// === OHT2 (4-20mA) Variables ===
const int levelPin = 34;          // ADC pin for OHT2
float oht2Percent = 0.0;
uint16_t oht2RawADC = 0;          // Raw ADC value
float oht2Voltage = 0.0;

const float EMPTY_VOLTAGE = 0.01;
const float FULL_VOLTAGE  = 1.15;

// === OHT1 Hydrostatic Constants ===
const float LEVEL_MAX = 1907.0;
const float LEVEL_MIN = 82.0;

#define SOLENOID_RELAY_PIN 4    // Change to your relay GPIO

SoftwareSerial mod(26, 27);

uint16_t calculateCRC(byte* buffer, int length) {
  uint16_t crc = 0xFFFF;
  for (int pos = 0; pos < length; pos++) {
    crc ^= (uint16_t)buffer[pos];
    for (int i = 8; i != 0; i--) {
      crc = (crc & 0x0001) ? (crc >> 1) ^ 0xA001 : (crc >> 1);
    }
  }
  return crc;
}

// ================== READ FLOWMETER ==================
bool readModbusFlowMeter(int fmIndex) {
  byte req[8];
  byte response[30] = {0};

  req[0] = fmIndex + 1;
  req[1] = 0x03; req[2] = 0x00; req[3] = 0x00;
  req[4] = 0x00; req[5] = 0x04;

  uint16_t crc = calculateCRC(req, 6);
  req[6] = crc & 0xFF;
  req[7] = crc >> 8;

  Serial.printf("FM%02d (Slave %d) ... ", fmIndex+1, req[0]);

  for (int attempt = 0; attempt < 5; attempt++) {
    while (mod.available()) mod.read();
    mod.write(req, 8);
    mod.flush();
    delay(400);

    int bytesRead = 0;
    unsigned long start = millis();
    while (mod.available() && bytesRead < 30 && millis() - start < 800) {
      response[bytesRead++] = mod.read();
    }

    if (bytesRead >= 13) {
      int adjust = (response[0] == req[0]) ? 0 : 1;
      if (response[0 + adjust] == req[0] && response[1 + adjust] == 0x03 && response[2 + adjust] == 0x08) {
        uint16_t calcCRC = calculateCRC(response + adjust, 11);
        uint16_t recvCRC = response[11 + adjust] | (response[12 + adjust] << 8);

        if (calcCRC == recvCRC) {
          totalizer[fmIndex] = ((int64_t)response[3+adjust]<<24) | ((int64_t)response[4+adjust]<<16) |
                               ((int64_t)response[5+adjust]<<8) | response[6+adjust];
          flowrate[fmIndex]  = ((int64_t)response[7+adjust]<<24) | ((int64_t)response[8+adjust]<<16) |
                               ((int64_t)response[9+adjust]<<8) | response[10+adjust];

          Serial.printf("OK | Flow:%d | Tot:%lld\n", flowrate[fmIndex], totalizer[fmIndex]);
          return true;
        }
      }
    }
    delay(150);
  }
  Serial.println("Failed");
  totalizer[fmIndex] = -1;
  flowrate[fmIndex] = -1;
  return false;
}

// ================== READ TDS ==================
float readTDS() {
  byte req[8] = {14, 0x03, 0x00, 0x03, 0x00, 0x01, 0, 0};
  uint16_t crc = calculateCRC(req, 6);
  req[6] = crc & 0xFF;
  req[7] = crc >> 8;

  while (mod.available()) mod.read();
  mod.write(req, 8);
  mod.flush();
  delay(400);

  byte resp[10] = {0};
  int len = 0;
  unsigned long start = millis();
  while (mod.available() && len < 10 && millis() - start < 600) resp[len++] = mod.read();

  if (len >= 7 && resp[1] == 0x03) {
    tdsValue = (resp[3] << 8) | resp[4];
    Serial.printf("TDS: %.0f\n", tdsValue);
    return tdsValue;
  }
  Serial.println("TDS Failed");
  return -1.0;
}

// ================== READ HYDROSTATIC OHT1 (Modbus) ==================
uint16_t readHydrostatic() {
  byte req[8] = {15, 0x03, 0x00, 0x04, 0x00, 0x01, 0, 0};
  uint16_t crc = calculateCRC(req, 6);
  req[6] = crc & 0xFF;
  req[7] = crc >> 8;

  while (mod.available()) mod.read();
  mod.write(req, 8);
  mod.flush();
  delay(400);

  byte resp[10] = {0};
  int len = 0;
  unsigned long start = millis();
  while (mod.available() && len < 10 && millis() - start < 600) resp[len++] = mod.read();

  if (len >= 7 && resp[1] == 0x03) {
    hydRawValue = (resp[3] << 8) | resp[4];

    if (hydRawValue <= LEVEL_MIN) oht1Percent = 0.0;
    else if (hydRawValue >= LEVEL_MAX) oht1Percent = 100.0;
    else {
      oht1Percent = ((float)(hydRawValue - LEVEL_MIN) / (LEVEL_MAX - LEVEL_MIN)) * 100.0;
    }

    Serial.printf("OHT1 Hydro Raw: %u | Percent: %.2f%%\n", hydRawValue, oht1Percent);
    return hydRawValue;
  }
  Serial.println("OHT1 Hydro Failed");
  return 0;
}

// ================== READ OHT2 (4-20mA Analog) ==================
void readOHT2() {
  oht2RawADC = analogRead(levelPin);
  oht2Voltage = (oht2RawADC * 3.3) / 4095.0;

  oht2Percent = ((oht2Voltage - EMPTY_VOLTAGE) / (FULL_VOLTAGE - EMPTY_VOLTAGE)) * 100.0;

  if (oht2Percent < 0) oht2Percent = 0;
  if (oht2Percent > 100) oht2Percent = 100;

  Serial.printf("OHT2 ADC: %u  Voltage: %.3f V  Level: %.1f%%\n", 
                oht2RawADC, oht2Voltage, oht2Percent);
}

// Send Functions
void sendFlowToServer(int i) {

  char data[128];
  snprintf(data, sizeof(data),
           "flowmeter_id:%s,flowrate:%d,totalizer:%lld,ver:2.0",
           flowmeter_id[i], flowrate[i], totalizer[i]);

  char url[300];
  snprintf(url, sizeof(url),
           "http://www.gowatr.com/wms/pushdata/hiranandani.php?id=hfm&data=%s",
           data);

  // Print exactly what is being sent
  Serial.println("\n----- Flowmeter Upload -----");
  Serial.println(data);
  // OR use: Serial.println(url);  // Prints the complete URL

  HTTPClient http;
  http.begin(url);
  int httpCode = http.GET();

  Serial.printf("HTTP Response: %d\n", httpCode);

  http.end();
}

void sendLevelToServer() {

  char data[180];
  snprintf(data, sizeof(data),
           "oht1:%.2f,oht1raw:%u,oht2:%.2f,oht2raw:%u,oht2volt:%.3f,tds1:%.2f,ver:2.0",
           oht1Percent, hydRawValue, oht2Percent,
           oht2RawADC, oht2Voltage, tdsValue);

  char url[300];
  snprintf(url, sizeof(url),
           "http://www.gowatr.com/wms/pushdata/hiranandani.php?id=hlvlm&data=%s",
           data);

  // Print exactly what is being sent
  Serial.println("\n----- Level Upload -----");
  Serial.println(data);
  // OR use: Serial.println(url);  // Prints the complete URL

  HTTPClient http;
  http.begin(url);
  int httpCode = http.GET();

  Serial.printf("HTTP Response: %d\n", httpCode);

  http.end();
}


void checkWiFi()
{
    static unsigned long lastReconnect = 0;
    static bool wifiConnected = false;

    if (WiFi.status() == WL_CONNECTED)
    {
        if (!wifiConnected)
        {
            wifiConnected = true;
            Serial.println("=================================");
            Serial.println("WiFi Connected!");
            Serial.print("IP Address: ");
            Serial.println(WiFi.localIP());
            Serial.println("=================================");
        }
        return;
    }

    // WiFi disconnected
    if (wifiConnected)
    {
        wifiConnected = false;
        Serial.println("WiFi Disconnected!");
    }

    // Retry every 10 seconds
    if (millis() - lastReconnect >= 10000)
    {
        lastReconnect = millis();

        Serial.println("Reconnecting WiFi...");
        WiFi.disconnect();
        WiFi.begin(ssid, password);
    }
}

void controlSolenoid()
{
    static bool solenoidON = false;

    // Tank level below 70% -> Start filling
    if (!solenoidON && oht1Percent < 70.0)
    {
        digitalWrite(SOLENOID_RELAY_PIN, HIGH);   // Relay ON
        solenoidON = true;
        Serial.println("Solenoid ON - Filling Tank");
    }

    // Tank level reaches 90% -> Stop filling
    else if (solenoidON && oht1Percent >= 90.0)
    {
        digitalWrite(SOLENOID_RELAY_PIN, LOW);    // Relay OFF
        solenoidON = false;
        Serial.println("Solenoid OFF - Tank Full");
    }
}

// ================== SETUP & LOOP ==================
void setup() {
  Serial.begin(115200);
  mod.begin(9600);
  analogReadResolution(12);   // 12-bit for OHT2
  pinMode(SOLENOID_RELAY_PIN, OUTPUT);
  digitalWrite(SOLENOID_RELAY_PIN, HIGH);   // Solenoid OFF
  //delay(2000);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.println("Connecting WiFi...");
}

void loop() {
  // Check WiFi status and reconnect if needed
  checkWiFi();

  Serial.println("\n=== Reading All Sensors ===");

  // Read all flowmeters
  for (int i = 0; i < NUM_FLOWMETERS; i++) {
    readModbusFlowMeter(i);
    delay(300);
  }

  // Read other sensors
  tdsValue = readTDS();
  delay(300);
  readHydrostatic();
  readOHT2();
  if (readHydrostatic() != -1)
    {
        controlSolenoid();
    }
    else
    {
        Serial.println("Hydro Failed - Previous Value Used");
    }

  // Upload data only if WiFi is connected
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\n=== Uploading Data ===");

    for (int i = 0; i < NUM_FLOWMETERS; i++) {
      sendFlowToServer(i);
      delay(600);
    }

    sendLevelToServer();
  } else {
    Serial.println("\nWiFi Offline - Upload Skipped");
  }

  Serial.println("=== Cycle Complete ===\n");
  delay(5000);
}