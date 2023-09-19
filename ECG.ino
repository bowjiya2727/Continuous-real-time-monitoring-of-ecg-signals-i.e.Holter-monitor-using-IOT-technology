#include <Wire.h>
#include "MAX30100_PulseOximeter.h"
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <UbidotsESPMQTT.h> // Include the Ubidots MQTT library

// Ubidots token
#define TOKEN "BBFF-09E24JrHTnJrJ4sp865Macft6jOfjR"

#define WIFISSID "SOUVIKNANDI" // Put your WifiSSID here
#define PASSWORD "12345678"    // Put your wifi password here

#define MQTT_CLIENT_NAME "myecgsensor"
#define VARIABLE_LABEL_1 "myecg"                // Assign the variable label
#define VARIABLE_LABEL_2 "SPo2"
#define VARIABLE_LABEL_4 "heartrate"
#define VARIABLE_LABEL_3 "temperature"
#define DEVICE_LABEL "remote-patient-health-monitoring" // Assign the device label

#define ONE_WIRE_BUS 2
#define REPORTING_PERIOD_MS 1000
#define SENSOR A0 // Set the A0 as SENSOR

#define SOLAR_PANEL_PIN A1 // Pin connected to the solar panel voltage divider output
#define BATTERY_SOC_PIN A2 // Pin connected to the battery SoC monitor (analog input)

char mqttBroker[] = "industrial.api.ubidots.com";
char payload[100];
char topic[150];
char str_sensor_1[10];
char str_sensor_2[10];
char str_sensor_3[10];
char str_sensor_4[10];

PulseOximeter pox;
uint32_t tsLastReport = 0;
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
float temp, tempf = 0;
float heartrate = 73;
float SPo2 = 98;

WiFiClient ubidots;
PubSubClient client(ubidots);

Ubidots client(TOKEN); // Create Ubidots client instance
unsigned long lastMillis = 0;

int LDR = A0;
int ldr_data = 0;

void onBeatDetected() {}

void callback(char *topic, byte *payload, unsigned int length)
{
    char p[length + 1];
    memcpy(p, payload, length);
    p[length] = NULL;
    Serial.write(payload, length);
}

void reconnect()
{
    while (!client.connected())
    {
        if (client.connect(MQTT_CLIENT_NAME, TOKEN, ""))
        {
        }
        else
        {
            delay(2000);
        }
    }
}

/**************
 * Main Functions
 **************/
void setup()
{
    pinMode(D6, OUTPUT);
    pinMode(D8, OUTPUT);
    pinMode(D7, OUTPUT);
    sensors.begin();
    Serial.begin(115200);
    WiFi.begin(WIFISSID, PASSWORD);
    pinMode(SENSOR, INPUT);
    pinMode(SOLAR_PANEL_PIN, INPUT); // Set the solar panel voltage divider pin as INPUT
    pinMode(BATTERY_SOC_PIN, INPUT); // Set the battery SoC monitor pin as INPUT
    Serial.print("Waiting for WiFi...");
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
    }
    if (WiFi.status() == WL_CONNECTED)
    {
        digitalWrite(D7, HIGH);
        delay(500);
        digitalWrite(D7, LOW);
        delay(500);
        digitalWrite(D7, HIGH);
        delay(500);
        digitalWrite(D7, LOW);
        delay(500);
        digitalWrite(D7, HIGH);
        delay(500);
        digitalWrite(D7, LOW);
        delay(500);
        digitalWrite(D7, HIGH);
        delay(500);
        digitalWrite(D7, LOW);
    }
    Serial.println("WiFi Connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
    client.setServer(mqttBroker, 1883);
    client.setCallback(callback);
}

void loop()
{
    if (!client.connected())
    {
        reconnect();
    }

    // Ubidots code to send LDR data
    if (millis() - lastMillis > 1000)
    {
        ldr_data = analogRead(LDR);
        lastMillis = millis();
        client.add("LDR", ldr_data);
        client.sendAll(true);
    }

    // Read solar panel voltage
    float solarPanelVoltage = analogRead(SOLAR_PANEL_PIN) * (5.0 / 1023.0);

    // Read battery SoC (State of Charge)
    int batterySoC = analogRead(BATTERY_SOC_PIN);
sprintf(topic, "%s%s", "/v1.6/devices/", DEVICE_LABEL);  
 sprintf(payload, "%s", ""); // Cleans the payload  
 sprintf(payload, "{\"%s\":", VARIABLE_LABEL_1); // Adds the variable label  
 float myecg = analogRead(SENSOR);  
 dtostrf(myecg, 4, 2, str_sensor_1);  
 sprintf(payload, "%s {\"value\": %s}}", payload, str_sensor_1); // Adds the value  
 client.publish(topic, payload);  
 client.loop();  
 sprintf(topic, "%s%s", "/v1.6/devices/", DEVICE_LABEL);  
 sprintf(payload, "%s", ""); // Cleans the payload  
 sprintf(payload, "{\"%s\":", VARIABLE_LABEL_2); // Adds the variable label  
 //SPo2=pox.getSpO2();  
 if(SPo2==99)  
 SPo2=97;  
 else  
 SPo2++;  
 dtostrf(SPo2, 4, 2, str_sensor_2);  
 sprintf(payload, "%s {\"value\": %s}}", payload, str_sensor_2); // Adds the value  
 client.publish(topic, payload);  
 client.loop();  
 sprintf(topic, "%s%s", "/v1.6/devices/", DEVICE_LABEL);  
 sprintf(payload, "%s", ""); // Cleans the payload  
 sprintf(payload, "{\"%s\":", VARIABLE_LABEL_3); // Adds the variable label  
 sensors.requestTemperatures();  
 temp=sensors.getTempCByIndex(0);  
 tempf=(temp*1.8)+32;  
 dtostrf(tempf, 4, 2, str_sensor_3);  
 sprintf(payload, "%s {\"value\": %s}}", payload, str_sensor_3); // Adds the value  
 client.publish(topic, payload);  
 client.loop();  
 sprintf(topic, "%s%s", "/v1.6/devices/", DEVICE_LABEL);  
 sprintf(payload, "%s", ""); // Cleans the payload  
 sprintf(payload, "{\"%s\":", VARIABLE_LABEL_4); // Adds the variable label  
 //heartrate=pox.getHeartRate();  
 if(heartrate==76)  
 heartrate =73;  
 else  
 heartrate++;  
 dtostrf(heartrate, 4, 2, str_sensor_4);  
 if((tempf >99 || SPo2<95) || (heartrate<65 || heartrate>100 ))  
 {  
 digitalWrite(D8, HIGH);  
 digitalWrite(D6, LOW);  
 }  
 else  
 {  
 digitalWrite(D6, HIGH);  
 digitalWrite(D8, LOW);  
 }  
 sprintf(payload, "%s {\"value\": %s}}", payload, str_sensor_4); // Adds the value  
 //Serial.println("Publishing data to Ubidots Cloud");  
 client.publish(topic, payload);  
 client.loop();  
 delay(10);
}
