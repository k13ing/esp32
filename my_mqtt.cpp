

#include <WiFi.h>
#include <PubSubClient.h>
#include <my_mqtt.h>
#include "DHTesp.h"
#include "ArduinoJson.h"

// Update these with values suitable for your network.

const char *ssid = "xiaomi";
const char *password = "123456789";
const char *mqtt_server = "broker.emqx.io";
extern float MQ_2_value;
WiFiClient espClient;
PubSubClient client(espClient);
extern bool window_status;
extern TempAndHumidity newValues ; // 获取温湿度
extern float lux;
extern void window_control(bool status);
#define MSG_BUFFER_SIZE (350)
char msg[MSG_BUFFER_SIZE];
int value = 0;

void setup_wifi()
{

    delay(10);
    // We start by connecting to a WiFi network
    Serial.println();
    Serial.print("Connecting to ");
    Serial.println(ssid);

    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);

    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
        Serial.print(".");
    }

    randomSeed(micros());

    Serial.println("");
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
}

void callback(char *topic, byte *payload, unsigned int length)
{

    Serial.print("Message arrived [");
    Serial.print(topic);
    Serial.print("] ");

    // 将 payload 转换为字符串
    char message[length + 1];
    for (unsigned int i = 0; i < length; i++)
    {
        message[i] = (char)payload[i];
    }
    message[length] = '\0';

    Serial.println(message);

    // 解析 JSON 消息
    StaticJsonDocument<200> doc;
    DeserializationError error = deserializeJson(doc, message);

    if (error)
    {
        Serial.print(F("deserializeJson() failed: "));
        Serial.println(error.f_str());
        return;
    }

    // 检查是否存在 "target" 字段
    if (doc.containsKey("target"))
    {
        // 获取 "target" 字段的值
        const char *target = doc["target"];

        // 检查是否为 "mod"
        if (strcmp(target, "window") == 0)
        {
            // 获取 "value" 字段的值
            int value = doc["value"];
            if (value == 1)
            {
                window_control(true);
            }
            else
            {
                window_control(false);
            }
        }
        else if (strcmp(target, "beep") == 0)
        {
            // 获取 "value" 字段的值
            int value = doc["value"];
            if (value == 1)
            {
                digitalWrite(BEEP, HIGH);
            }
            else
            {
                digitalWrite(BEEP, LOW);
            }

        }
        else if (strcmp(target, "led") == 0)
        {
            // 获取 "value" 字段的值
            int value = doc["value"];
            if (value == 1)
            {
               digitalWrite(LED1, HIGH);
            }
            else
            {
                digitalWrite(LED1, LOW);
            }
        }
        else if (strcmp(target, "fan") == 0)
        {
            // 获取 "value" 字段的值
            int value = doc["value"];
            if (value == 1)
            {
                digitalWrite(MOTOR, HIGH);
            }
            else
            {
                digitalWrite(MOTOR, LOW);
            }

        }
        mqtt_publish();
    }
    else
    {
        Serial.println("No 'target' field found");
    }
}

void reconnect()
{
    // Loop until we're reconnected
    while (!client.connected())
    {
        Serial.print("Attempting MQTT connection...");
        // Create a random client ID
        String clientId = "ESP8266Client-";
        clientId += String(random(0xffff), HEX);
        // Attempt to connect
        if (client.connect(clientId.c_str()))
        {
            Serial.println("connected");
            // Once connected, publish an announcement...
            //client.publish(topic_pub, "hello world"); // 发布
            // ... and resubscribe
            client.subscribe(topic_sub); // 订阅
        }
        else
        {
            Serial.print("failed, rc=");
            Serial.print(client.state());
            Serial.println(" try again in 5 seconds");
            // Wait 5 seconds before retrying
            delay(5000);
        }
    }
}

void mqtt_init()
{
    pinMode(BUILTIN_LED, OUTPUT); // Initialize the BUILTIN_LED pin as an output
    setup_wifi();
    client.setServer(mqtt_server, 1883);
    client.setCallback(callback);
}

// void loop() {
//  if (!client.connected()) {
//     reconnect();
//   }
//   client.loop();
//   unsigned long now = millis();
//   if (now - lastMsg > 2000) {
//     lastMsg = now;
//     ++value;
//     snprintf (msg, MSG_BUFFER_SIZE, "hello world #%ld", value);
//     Serial.print("Publish message: ");
//     Serial.println(msg);
//     client.publish("outTopic", msg);
//   }
// }

void mqtt_publish()
{

    bool beep_status = digitalRead(BEEP);
    bool fan_status = digitalRead(MOTOR);
    bool led_status = digitalRead(LED1);

    sprintf(msg, "{\"window\":%d,\"temp\":%0.2f,\"humi\":%0.2f,\"light\":%0.2f,\"led\":%d,\"beep\":%d,\"fan\":%d,\"gas\":%0.2f}", window_status,
            newValues.temperature, newValues.humidity, lux, led_status, beep_status,fan_status,MQ_2_value);

    client.publish(topic_pub, msg);
}
void mqtt_reconnect()
{
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
}