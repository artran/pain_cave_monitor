#include <Arduino.h>
/*
Connecting the BME280 Sensor:
Sensor              ->  Board
-----------------------------
Vin (Voltage In)    ->  3.3V
Gnd (Ground)        ->  Gnd
SDA (Serial Data)   ->  A4 on Uno/Pro-Mini, 20 on Mega2560/Due, 2 Leonardo/Pro-Micro
SCK (Serial Clock)  ->  A5 on Uno/Pro-Mini, 21 on Mega2560/Due, 3 Leonardo/Pro-Micro

 */

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <BME280I2C.h>
#include <PubSubClient.h>
#include <sstream>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>

#define SERIAL_BAUD 115200

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET     4 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#define NUMFLAKES     10 // Number of snowflakes in the animation example

#define LOGO_HEIGHT   16
#define LOGO_WIDTH    16
static const unsigned char PROGMEM logo_bmp[] =
        { B00000000, B11000000,
          B00000001, B11000000,
          B00000001, B11000000,
          B00000011, B11100000,
          B11110011, B11100000,
          B11111110, B11111000,
          B01111110, B11111111,
          B00110011, B10011111,
          B00011111, B11111100,
          B00001101, B01110000,
          B00011011, B10100000,
          B00111111, B11100000,
          B00111111, B11110000,
          B01111100, B11110000,
          B01110000, B01110000,
          B00000000, B00110000 };

const char* ssid = "artran";
const char* wifiPassword = "Life is like a candle";

const char* mqttServer = "m24.cloudmqtt.com";
const int mqttPort = 18919;
const char* mqttUser = "aqgqghbc";
const char* mqttPassword = "zvFEUZexwzp5";

BME280I2C bme;    // Default : forced mode, standby time = 1000 ms
                  // Oversampling = pressure ×1, temperature ×1, humidity ×1, filter off,

WiFiClient espClient;
PubSubClient client(espClient);

void printBME280Data(Stream* serial_stream);

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

    Serial.println("Connected to the WiFi network");
    WiFi.enableIpV6();
    Serial.println(WiFi.localIP());

    while(!bme.begin()) {
        Serial.println("Could not find BME280 sensor!");
        delay(1000);
    }

    switch(bme.chipModel()) {
        case BME280::ChipModel_BME280:
            Serial.println("Found BME280 sensor! Success.");
            break;
        case BME280::ChipModel_BMP280:
            Serial.println("Found BMP280 sensor! No Humidity available.");
            break;
        default:
            Serial.println("Found UNKNOWN sensor! Error!");
    }

    // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
    if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3D for 128x64
        Serial.println(F("SSD1306 allocation failed"));
        for(;;); // Don't proceed, loop forever
    }

    // Show initial display buffer contents on the screen --
    // the library initializes this with an Adafruit splash screen.
    display.display();

    client.setServer(mqttServer, mqttPort);

    while (!client.connected()) {
        Serial.println("Connecting to MQTT...");

        if (client.connect("ESP32Client", mqttUser, mqttPassword )) {

            Serial.println("connected");

        } else {
            Serial.print("failed with state ");
            Serial.print(client.state());
            delay(2000);

        }
    }
}

void loop() {
//    Serial.println(WiFi.localIPv6());
    printBME280Data(&Serial);
    delay(500);
}

void printBME280Data(Stream* serial_stream) {
    float temp(NAN), hum(NAN), pres(NAN);
    std::ostringstream oss;

    BME280::TempUnit tempUnit(BME280::TempUnit_Celsius);
    BME280::PresUnit presUnit(BME280::PresUnit_Pa);

    bme.read(pres, temp, hum, tempUnit, presUnit);

    serial_stream->print("Temp: ");
    serial_stream->print(temp);
    serial_stream->print("°"+ String(tempUnit == BME280::TempUnit_Celsius ? 'C' :'F'));
    serial_stream->print("\t\tHumidity: ");
    serial_stream->print(hum);
    serial_stream->print("% RH");
    serial_stream->print("\t\tPressure: ");
    serial_stream->print(pres);
    serial_stream->println(" Pa");

    oss << "{\"temp\": " << temp << ", \"humidity\": " << hum << ", \"pressure\": " << pres << "}";

    client.publish("paincavemonitor", oss.str().c_str());

    delay(60 * 1000);
}
