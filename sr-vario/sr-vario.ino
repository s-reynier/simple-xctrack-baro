#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <Wire.h>
#include <Adafruit_BMP280.h>

Adafruit_BMP280 bmp; // use I2C interface

// See the following for generating UUIDs:
// https://www.uuidgenerator.net/

#define SERVICE_UUID        "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"
#define BLE_SERVER_NAME "SR-VARIO"

BLECharacteristic *pCharacteristic;
#define BT_PACKET_SIZE 20


void setup() {
  Serial.begin(115200);
  Serial.println("Starting BLE work!");

  unsigned status;
  status = bmp.begin(0x76);
  if (!status) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring or "
                      "try a different address!"));
    while (1) delay(10);
  }

  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
                  
  BLEDevice::init(BLE_SERVER_NAME);
  BLEServer *pServer = BLEDevice::createServer();
  BLEService *pService = pServer->createService(SERVICE_UUID);
  
  pCharacteristic = pService->createCharacteristic(
                                         CHARACTERISTIC_UUID,
                                         BLECharacteristic::PROPERTY_NOTIFY
                                       );

  pService->start();
  
  // BLEAdvertising *pAdvertising = pServer->getAdvertising();  // this still is working for backward compatibility
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  
  BLEDevice::startAdvertising();
  Serial.println("Characteristic defined! Now you can read it in your phone!");
}


static uint8_t ble_uart_nmea_checksum(const char *szNMEA){
  const char* sz = &szNMEA[1]; // skip leading '$'
  uint8_t cksum = 0;
  while ((*sz) != 0 && (*sz != '*')) {
    cksum ^= (uint8_t) *sz;
    sz++;
    }
  return cksum;
  }
  
// $LXWP0,N,,598.9,0.02,,,,,,,,*52  
// $LK8EX1,101133,99999,22,36,1000,*2F
void sendTrameLK8EX1()
{

    char szmsg[40];
    //https://github.com/LK8000/LK8000/blob/master/Docs/LK8EX1.txt
    sprintf(szmsg, "$LK8EX1,%d,%d,%d,99,999*", 101111,99999, 9999);
    uint8_t cksum = ble_uart_nmea_checksum(szmsg);
    char szcksum[5];
    sprintf(szcksum,"%02X\r\n", cksum);
    strcat(szmsg, szcksum);

    Serial.print(szmsg);
    pCharacteristic->setValue(szmsg);
    pCharacteristic->notify();   
}

void loop() {

   Serial.print(F("Temperature = "));
    Serial.print(bmp.readTemperature());
    Serial.println(" *C");

    Serial.print(F("Pressure = "));
    Serial.print(bmp.readPressure());
    Serial.println(" Pa");

    //Serial.print(F("Approx altitude = "));
    //Serial.print(bmp.readAltitude(1032)); /* Adjusted to local forecast! */
    //Serial.println(" m");

    Serial.println();


  
  // put your main code here, to run repeatedly:
  sendTrameLK8EX1();
  delay(1000);
}
