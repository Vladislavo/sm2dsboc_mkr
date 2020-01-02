#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_ADS1015.h>

#define BUS_PROTOCOL_TRANSMIT_RETRIES       5
#define BUS_PROTOCOL_MAX_DATA_SIZE          32
#define DEBUG_SERIAL_RX_PIN                 2
#define DEBUG_SERIAL_TX_PIN                 3

#define BAUDRATE                            115200
#define DHT22_PIN                           A6

#define DATA_SEND_PERIOD                    2000

Adafruit_ADS1115 ads;

void setup() {
    Serial.begin(BAUDRATE);
    Wire.begin();

    ads.begin();
}

void loop() {
    int16_t adc0, adc1, adc2;

    adc0 = ads.readADC_SingleEnded(0);
    adc1 = ads.readADC_SingleEnded(1);
    adc2 = ads.readADC_SingleEnded(2);

    delay(1000);
}