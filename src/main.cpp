#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_ADS1015.h>

#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>

#include "bus_protocol/bus_protocol.h"

#define BUS_PROTOCOL_MAX_WAITING_TIME       300
#define BUS_PROTOCOL_TRANSMIT_RETRIES       5
#define BUS_PROTOCOL_MAX_DATA_SIZE          32
#define DEBUG_SERIAL_RX_PIN                 2
#define DEBUG_SERIAL_TX_PIN                 3

#define BAUDRATE                            115200
#define DHT22_PIN                           A1

#define DATA_SEND_PERIOD                    2000

typedef struct {
    board_id_t board_id = BUS_PROTOCOL_BOARD_ID_MKR;
    uint32_t utc = 0;
    uint16_t soil_moisture_0 = 0;
    uint16_t soil_moisture_1 = 0;
    uint16_t soil_moisture_2 = 0;
    float dht_temp = 0;
    float dht_hum = 0;
} sensors_data_t;

Adafruit_ADS1115 ads;
DHT dht(DHT22_PIN, DHT22);

void read_sensors_data(sensors_data_t *sensors_data);
uint8_t send_data(const sensors_data_t *sensors_data);

uint8_t bus_protocol_serial_receive(uint8_t *data, uint8_t *data_length, uint32_t timeout);

void sleep_mcu(uint32_t ms);

sensors_data_t sensors_data;

void setup() {
    Serial.begin(BAUDRATE);
    Serial1.begin(BAUDRATE);
    Wire.begin();

    ads.begin();
    dht.begin();
}

void loop() {
    read_sensors_data(&sensors_data);

    send_data(&sensors_data);

    sleep_mcu(DATA_SEND_PERIOD + (rand() % 2000));
}

void read_sensors_data(sensors_data_t *sensors_data) {
    sensors_data->soil_moisture_0 = ads.readADC_SingleEnded(0);
    sensors_data->soil_moisture_1 = ads.readADC_SingleEnded(1);
    sensors_data->soil_moisture_2 = ads.readADC_SingleEnded(2);

    float h = dht.readHumidity();
    float t = dht.readTemperature();

    sensors_data->dht_temp = t;
    sensors_data->dht_hum = h;

    if (isnan(h) || isnan(t)) {
      Serial.println(F("Failed to read from DHT sensor!"));
      return;
    }

    Serial.print(F("Humidity: "));
    Serial.print(h);
    Serial.print(F("%  Temperature: "));
    Serial.print(t);
    Serial.print(F("°C "));
}

uint8_t send_data(const sensors_data_t *sensors_data) {
    uint8_t ret = 0;
    uint8_t retries = 0;

    uint8_t packet_buffer[32] = {0};
    uint8_t packet_buffer_length = 0;

    // send
    // TODO: integrate time
    bus_protocol_data_send_encode(  (uint8_t *) sensors_data,
                                    sizeof(*sensors_data),
                                    packet_buffer,
                                    &packet_buffer_length);

    do {
        Serial1.write(packet_buffer, packet_buffer_length);

        //wait for ACK
        if (bus_protocol_serial_receive(packet_buffer, &packet_buffer_length, 1000) &&
            bus_protocol_packet_decode( packet_buffer, 
                                        packet_buffer_length, 
                                        packet_buffer, 
                                        &packet_buffer_length) == BUS_PROTOCOL_PACKET_TYPE_ACK) 
        {
            Serial.println("ESP ACK");
            ret = 1;
        } else {
            retries++;
        }

    } while(!ret && retries < BUS_PROTOCOL_TRANSMIT_RETRIES);

    return ret;
}

uint8_t bus_protocol_serial_receive(uint8_t *data, uint8_t *data_length, uint32_t timeout) {
    *data_length = 0;
    uint32_t start_millis = millis();
    while(start_millis + timeout > millis() && *data_length < BUS_PROTOCOL_MAX_DATA_SIZE) {
        if (Serial1.available()) {
            data[(*data_length)++] = Serial1.read();
            // update wating time
            start_millis = millis();
        }
    }

    // Serial.print(F("Received message: "));
    // for (uint8_t i = 0; i < *data_length; i++) {
    //     Serial.write(data[i]);
    // }
    // Serial.println();

    return *data_length;
}

void sleep_mcu(uint32_t ms) {
    // future possible employ powerdown functions
    delay(ms);
}