#include <Arduino.h>

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <BME280I2C.h>
#include <PubSubClient.h>
#include <sstream>
#include <WiFi.h>
#include <esp_adc_cal.h>

#define SERIAL_BAUD 115200

#define OLED_RESET    -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

#define SAMPLES_PER_READING 64  // How many ADC samples to take per reading

const int SAMPLE_EVERY_SECS = 60;  // How many seconds betwen samples
const uint8_t BEEP = 2;  // GPIO used for beeper
const uint8_t ALERT_LED = 0;  // GPIO used for red LED
const uint8_t ACTIVITY_LED = 4;  // GPIO used for blue LED

const uint8_t BATTERY_CURRENT_1 = 36; // GPIO used for current sensor input 1
const uint8_t BATTERY_CURRENT_2 = 39; // GPIO used for current sensor input 2
const uint8_t BATTERY_VOLTAGE = 34;  // GPIO used for battery voltage input
const uint16_t HALL_1_ZERO_OFFSET = 2248;
const uint16_t HALL_2_ZERO_OFFSET = 2287;

uint32_t v1_total = 0;
uint32_t v2_total = 0;
int sample_count = 0;

typedef struct {
    float humidity;
    float pressure;
    float temperature;
    double battery_voltage;
    double battery_current;
    double battery_power;
    double battery_remaining;
} sensor_data;

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
PubSubClient mqtt_client(espClient); /* NOLINT */

volatile boolean interrupt_flag = false;
hw_timer_t * timer = nullptr;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
esp_adc_cal_characteristics_t adc_cal_value;

// Initialisation
void initialise_gpio();

void initialise_display();

void initialise_wifi();

void initialise_mqtt();

void initialise_bme280();

void initialise_timer_interrupt();

// Normal operation
void handle_timer_interrupt();

sensor_data fetch_bme280_data();

double measure_battery_current();

double measure_battery_voltage();

double calculate_remaining_capacity();

void update_alarms(sensor_data data);

void print_sensor_data(sensor_data *data);

void display_sensor_data(sensor_data *data);

void publish_sensor_data(sensor_data *data);

void setup() {
    Serial.begin(SERIAL_BAUD);
    while(!Serial) {} // Wait
    Serial.println("########## Initialisation ##########");

    initialise_gpio();

    digitalWrite(ALERT_LED, HIGH);

    initialise_display();

    initialise_wifi();

    initialise_mqtt();

    initialise_bme280();

    initialise_timer_interrupt();
    interrupt_flag = true;  // Do one scan immediately

    Serial.println("####################################\n");
    digitalWrite(ALERT_LED, LOW);
}

void initialise_gpio() {
    pinMode(BEEP, OUTPUT);
    pinMode(ALERT_LED, OUTPUT);
    pinMode(ACTIVITY_LED, OUTPUT);

    pinMode(BATTERY_CURRENT_1, ANALOG);
    pinMode(BATTERY_CURRENT_2, ANALOG);
    pinMode(BATTERY_VOLTAGE, ANALOG);

    digitalWrite(BEEP, LOW);
    digitalWrite(ALERT_LED, LOW);
    digitalWrite(ACTIVITY_LED, LOW);

    esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, 1100, &adc_cal_value);
    Serial.println("\n########## ADC Calibration ##########");
    Serial.printf("CoeffA: %u\tCoeffB %u\n", adc_cal_value.coeff_a, adc_cal_value.coeff_b);
    Serial.println("#####################################\n");
}

void initialise_display() {
    if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
        Serial.println(F("SSD1306 allocation failed"));
        for(;;); // Don't proceed, loop forever
    }

    display.display();
}

void initialise_wifi() {
    digitalWrite(ACTIVITY_LED, HIGH);

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

    digitalWrite(ACTIVITY_LED, LOW);
}

void initialise_mqtt() {
    mqtt_client.setServer(mqttServer, mqttPort);
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

void initialise_timer_interrupt() {
    timer = timerBegin(0, 80, true);
    timerAttachInterrupt(timer, &handle_timer_interrupt, true);
    timerAlarmWrite(timer, SAMPLE_EVERY_SECS * 1000000, true);
    timerAlarmEnable(timer);
}

void IRAM_ATTR handle_timer_interrupt() {
    portENTER_CRITICAL_ISR(&timerMux);
    interrupt_flag = true;
    portEXIT_CRITICAL_ISR(&timerMux);
}

void loop() {
    if (interrupt_flag) {
        portENTER_CRITICAL(&timerMux);
        interrupt_flag = false;
        portEXIT_CRITICAL(&timerMux);

        digitalWrite(ACTIVITY_LED, HIGH);
        sensor_data data = fetch_bme280_data();
        data.battery_current = measure_battery_current();
        data.battery_voltage = measure_battery_voltage();
        data.battery_power = data.battery_voltage * data.battery_current;
        data.battery_remaining = calculate_remaining_capacity();

        print_sensor_data(&data);
        display_sensor_data(&data);
        publish_sensor_data(&data);

        delay(250);
        digitalWrite(ACTIVITY_LED, LOW);

        delay((SAMPLE_EVERY_SECS - 2) * 1000);
    }

}

/**
 * Get the environmental data from the BME280 and return it in a sensor_data
 *
 * @return a sensor_data object with the environmental values filled in
 */
sensor_data fetch_bme280_data() {
    sensor_data data = sensor_data();
    bme.read(data.pressure, data.temperature, data.humidity);
    return data;
}

/**
 * Read the two hall effect sensors and calculate the total current being drawn
 *
 * Note: both sensors output 2.14V for zero current.
 *
 * The sensors are wired so that the signal *decrease* for *increasing* current.
 *
 * Full scale is 30A for each sensor, sensitivity is 66mV/A.
 *
 * @return the total current being drawn from the battery
 */
double measure_battery_current() {
    uint32_t hall_1 = 0;
    uint32_t hall_2 = 0;

    for (int i = 0; i < SAMPLES_PER_READING; i++) {
        hall_1 += analogRead(BATTERY_CURRENT_1);
        hall_2 += analogRead(BATTERY_CURRENT_2);
    }

    hall_1 /= SAMPLES_PER_READING;
    hall_2 /= SAMPLES_PER_READING;

    int voltage_1 = esp_adc_cal_raw_to_voltage(hall_1, &adc_cal_value) - HALL_1_ZERO_OFFSET;
    int voltage_2 = esp_adc_cal_raw_to_voltage(hall_2, &adc_cal_value) - HALL_2_ZERO_OFFSET;

    // TODO: remove after testing
    v1_total += esp_adc_cal_raw_to_voltage(hall_1, &adc_cal_value);
    v2_total += esp_adc_cal_raw_to_voltage(hall_2, &adc_cal_value);
    sample_count++;
    Serial.printf("V1: %d mV\t\tV2: %d mV\n", voltage_1, voltage_2);
    Serial.printf("Av V1: %u\t\tAv V2: %u\t\tSamples: %d\n", v1_total/sample_count, v2_total/sample_count, sample_count);
    // END TODO

    double current1 = voltage_1 / 66.0;
    double current2 = voltage_2 / 66.0;

    return current1 + current2;
}

/**
 * Read the voltage divider and calculate the battery voltage from it.
 * Voltage divider is 3k3 to ground, 10k to battery
 *
 * @return the battery voltage
 */
double measure_battery_voltage() {
    uint32_t raw_batt_in = 0;

    for (int i = 0; i < SAMPLES_PER_READING; i++) {
        raw_batt_in += analogRead(BATTERY_VOLTAGE);
    }

    raw_batt_in /= SAMPLES_PER_READING;
    uint32_t battery_in = esp_adc_cal_raw_to_voltage(raw_batt_in, &adc_cal_value);

    return (battery_in/1000.0) * 13300 / 3300;
}

/**
 * Calculate how much battery remains based on the current voltage and current drain
 *
 * @return an estimate of the number of minutes left at the present load
 */
double calculate_remaining_capacity() {
    return 0; // TODO
}

void print_sensor_data(sensor_data *data) {
    Serial.printf("Temp: %.2f°C\t\tHumidity: %.2f%% RH\t\tPressure: %.2f Pa\n",
            data->temperature, data->humidity, data->pressure);
    Serial.printf("Voltage: % .1f V\tCurrent: % .1f A\t\tPower: % .1f W\t\t Remaining: %.0f mins\n\n",
            data->battery_voltage, data->battery_current, data->battery_power, data->battery_remaining);
}

void display_sensor_data(sensor_data *data) {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(WHITE);

    const auto half_height = static_cast<const int16_t>(display.height() / 2);
    const auto half_width = static_cast<const int16_t>(display.width() / 2);

    const int16_t left_column = 10;
    const auto right_column = static_cast<const int16_t>(left_column + half_width );
    const int16_t top_row = 10;
    const auto bottom_row = static_cast<const int16_t>(top_row + half_height);

    display.drawFastHLine(0, half_height, display.width(), WHITE);
    display.drawFastVLine(half_width, 0, display.height(), WHITE);

    display.setCursor(left_column, top_row);
    display.printf("%.1f%cC", data->temperature, (char)247);

    display.setCursor(right_column, top_row);
    display.printf("%.1f%% RH", data->humidity);

    display.setCursor(left_column, bottom_row);
    display.printf("% .1f V", data->battery_voltage);

    display.setCursor(right_column, bottom_row);
    display.printf("% .1f A", data->battery_current);

    display.display();
}

void publish_sensor_data(sensor_data *data) {
    std::ostringstream oss;
    if (!mqtt_client.connected()) {
        if (!mqtt_client.connect("PainCaveMonitorClient", mqttUser, mqttPassword)) {
            Serial.print("Connection to MQTT broker failed with state: ");
            Serial.println(mqtt_client.state());
            delay(2000);
        }
    }

    if (mqtt_client.connected()) {
        oss << "{\"temperature\": " << data->temperature << ", \"humidity\": " << data->humidity << ", \"pressure\": "
            << data->pressure << ", \"voltage\": " << data->battery_voltage << ", \"current\": "
            << data->battery_current << ", \"power\": " << data->battery_power << ", \"remaining\": "
            << data->battery_remaining << "}";
        const char *payload = oss.str().c_str();
//        Serial.print("MQTT payload: ");
//        Serial.println(payload);
        mqtt_client.publish("paincavemonitor", payload);
        mqtt_client.disconnect();
    }
}

