#include <Arduino.h>

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <BME280I2C.h>
#include <PubSubClient.h>
#include <sstream>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>

#define SERIAL_BAUD 115200

#define OLED_RESET    -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

struct sensor_data {
    float humidity;
    float pressure;
    float temperature;
};

const char* ssid = "artran";
const char* wifiPassword = "Life is like a candle";

const char* mqttServer = "m24.cloudmqtt.com";
const int mqttPort = 18919;
const char* mqttUser = "aqgqghbc";
const char* mqttPassword = "zvFEUZexwzp5";

BME280I2C bme;    // Default : forced mode, standby time = 1000 ms /* NOLINT */
                  // Oversampling = pressure ×1, temperature ×1, humidity ×1, filter off,
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET); /* NOLINT */
WiFiClient espClient; /* NOLINT */
PubSubClient client(espClient); /* NOLINT */

void initialise_display();

void initialise_wifi();

void initialise_bme280();

void initialise_mqtt();

void print_sensor_data(sensor_data data);

sensor_data fetch_bme280_data();

void publish_sensor_data(sensor_data data);

#pragma clang diagnostic push
#pragma ide diagnostic ignored "OCUnusedGlobalDeclarationInspection"
void setup() {
    Serial.begin(SERIAL_BAUD);
    while(!Serial) {} // Wait

    initialise_display();

    initialise_bme280();

    initialise_wifi();

    initialise_mqtt();
}
#pragma clang diagnostic pop

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wmissing-noreturn"
void initialise_display() {
    if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
        Serial.println(F("SSD1306 allocation failed"));
        for(;;); // Don't proceed, loop forever
    }

    display.display();
}
#pragma clang diagnostic pop

void initialise_wifi() {
    WiFi.disconnect(true);
    WiFi.begin(ssid, wifiPassword);
    Wire.begin();

    while (!WiFi.isConnected()) {
        delay(1000);
        Serial.println("Connecting to WiFi..");
    }

    Serial.print("Connected to the WiFi network with IP: ");
    Serial.println(WiFi.localIP());
    WiFi.enableIpV6();
}

void initialise_bme280() {
    while(!bme.begin()) {
        Serial.println("Could not find BME280 sensor!");
        delay(1000);
    }

    switch(bme.chipModel()) {
        case BME280::ChipModel_BME280:
            Serial.println("Found BME280 sensor!");
            break;
        case BME280::ChipModel_BMP280:
            Serial.println("Found BMP280 sensor! No Humidity available.");
            break;
        default:
            Serial.println("Found UNKNOWN sensor! Error!");
    }
}

void initialise_mqtt() {
    client.setServer(mqttServer, mqttPort);
}

#pragma clang diagnostic push
#pragma ide diagnostic ignored "OCUnusedGlobalDeclarationInspection"
void loop() {
    sensor_data bme_data = fetch_bme280_data();
    print_sensor_data(bme_data);
    publish_sensor_data(bme_data);
    delay(60 * 1000);
}
#pragma clang diagnostic pop


sensor_data fetch_bme280_data() {
    sensor_data data = sensor_data();
    bme.read(data.pressure, data.temperature, data.humidity);
    return data;
}

void print_sensor_data(sensor_data data) {
    Serial.print("Temp: ");
    Serial.print(data.temperature);
    Serial.print("°C");
    Serial.print("\t\tHumidity: ");
    Serial.print(data.humidity);
    Serial.print("% RH");
    Serial.print("\t\tPressure: ");
    Serial.print(data.pressure);
    Serial.println(" Pa");
}

void publish_sensor_data(sensor_data data) {
    std::ostringstream oss;
    if (!client.connected()) {
        if (!client.connect("PainCaveMonitorClient", mqttUser, mqttPassword)) {
            Serial.print("Connection to MQTT broker failed with state: ");
            Serial.println(client.state());
            delay(2000);
        }
    }

    if (client.connected()) {
        oss << "{\"temperature\": " << data.temperature << ", \"humidity\": " << data.humidity << ", \"pressure\": " << data.pressure << "}";
        client.publish("paincavemonitor", oss.str().c_str());
    }
}

