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

#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels


const char* ssid = "artran";
const char* wifiPassword = "Life is like a candle";

const char* mqttServer = "m24.cloudmqtt.com";
const int mqttPort = 18919;
const char* mqttUser = "aqgqghbc";
const char* mqttPassword = "zvFEUZexwzp5";

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
BME280I2C bme;    // Default : forced mode, standby time = 1000 ms
                  // Oversampling = pressure ×1, temperature ×1, humidity ×1, filter off,

WiFiClient espClient;
PubSubClient client(espClient);

void printBME280Data(Stream* serial_stream);

#pragma clang diagnostic push
#pragma ide diagnostic ignored "OCUnusedGlobalDeclarationInspection"
#pragma clang diagnostic ignored "-Wmissing-noreturn"
void setup() {
    Serial.begin(SERIAL_BAUD);
    WiFi.disconnect(true);
    WiFi.begin(ssid, wifiPassword);
    Wire.begin();

    while(!Serial) {} // Wait

    while (!WiFi.isConnected()) {
        delay(1000);
        Serial.println("Connecting to WiFi..");
    }

    Serial.print("Connected to the WiFi network with IP: ");
    Serial.println(WiFi.localIP());
    WiFi.enableIpV6();

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

    if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3D for 128x64
        Serial.println(F("SSD1306 allocation failed"));
        for(;;); // Don't proceed, loop forever
    }

    display.display();

    client.setServer(mqttServer, mqttPort);
}
#pragma clang diagnostic pop

void loop() {
//    Serial.println(WiFi.localIPv6());
    printBME280Data(&Serial);
    delay(60 * 1000);
}

void printBME280Data(Stream* serial_stream) {
    float temperature(NAN), humidity(NAN), pressure(NAN);
    std::ostringstream oss;

    bme.read(pressure, temperature, humidity);

    serial_stream->print("Temp: ");
    serial_stream->print(temperature);
    serial_stream->print("°C");
    serial_stream->print("\t\tHumidity: ");
    serial_stream->print(humidity);
    serial_stream->print("% RH");
    serial_stream->print("\t\tPressure: ");
    serial_stream->print(pressure);
    serial_stream->println(" Pa");

    if (!client.connected()) {
        if (!client.connect("PainCaveMonitorClient", mqttUser, mqttPassword)) {
            Serial.print("Connection to MQTT broker failed with state: ");
            Serial.println(client.state());
            delay(2000);
        }
    }

    if (client.connected()) {
        oss << "{\"temperature\": " << temperature << ", \"humidity\": " << humidity << ", \"pressure\": " << pressure << "}";
        client.publish("paincavemonitor", oss.str().c_str());
    }
}
