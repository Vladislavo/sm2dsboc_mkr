#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_ADS1015.h>

#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>

#include "bus_protocol/bus_protocol.h"

#include "util/log/log.h"

#define LOG_LEVEL                           LOG_LEVEL_NONE

#define BUS_PROTOCOL_MAX_WAITING_TIME       300
#define BUS_PROTOCOL_TRANSMIT_RETRIES       5
#define BUS_PROTOCOL_MAX_DATA_SIZE          32
#define DEBUG_SERIAL_RX_PIN                 2
#define DEBUG_SERIAL_TX_PIN                 3

#define BAUDRATE                            115200
#define DHT22_PIN                           A1

#define DATA_SEND_PERIOD                    60000

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
uint8_t send_data(const sensors_data_t *sensors_data, const int8_t retries);

uint8_t bus_protocol_serial_receive(uint8_t *data, uint8_t *data_length, uint32_t timeout);

void sleep_mcu(uint32_t ms);

sensors_data_t sensors_data;

void setup() {
    Serial.begin(BAUDRATE);
    Serial1.begin(BAUDRATE);
    Wire.begin();

    ads.begin();
    dht.begin();

    randomSeed(ads.readADC_SingleEnded(0));
}

void loop() {
    do {
        delay(random(5000)); 
        read_sensors_data(&sensors_data);
    } while (!send_data(&sensors_data, BUS_PROTOCOL_TRANSMIT_RETRIES));

    sleep_mcu(DATA_SEND_PERIOD + random(5000));
}

void read_sensors_data(sensors_data_t *sensors_data) {
    sensors_data->soil_moisture_0 = ads.readADC_SingleEnded(0);
    sensors_data->soil_moisture_1 = ads.readADC_SingleEnded(1);
    sensors_data->soil_moisture_2 = ads.readADC_SingleEnded(2);

    do {
        sensors_data->dht_temp = dht.readTemperature();
        sensors_data->dht_hum = dht.readHumidity();
        if (isnan(sensors_data->dht_temp) || isnan(sensors_data->dht_hum)) {
            LOG_E(F("Failed to read from DHT sensor!\n"));
            delay(100);
        }
    } while (isnan(sensors_data->dht_temp) || isnan(sensors_data->dht_hum));

    LOG_D(F("Humidity: "));
    LOG_D(h);
    LOG_D(F("%  Temperature: "));
    LOG_D(t);
    LOG_D(F("°C "));
}

uint8_t send_data(const sensors_data_t *sensors_data, const int8_t retries) {
    uint8_t ret = 0;
    uint8_t retr = 0;

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
        if (bus_protocol_serial_receive(packet_buffer, &packet_buffer_length, 3000) &&
            bus_protocol_packet_decode( packet_buffer, 
                                        packet_buffer_length, 
                                        packet_buffer, 
                                        &packet_buffer_length) == BUS_PROTOCOL_PACKET_TYPE_ACK) 
        {
            LOG_D(F("ESP ACK"));
            ret = 1;
        } else {
            retr++;
        }

    } while(!ret && retr < retries);

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

    return *data_length;
}

void sleep_mcu(uint32_t ms) {
    // future possible employ powerdown functions
    delay(ms);
}