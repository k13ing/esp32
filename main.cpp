#include <Arduino.h>
#include <ESP32Servo.h>
#include <Wire.h> // Only needed for Arduino 1.6.5 and earlier
#include "SSD1306Wire.h"
#include "DHTesp.h"
#include <BH1750.h>
#include <my_mqtt.h>
#include <MQUnifiedsensor.h>

#define Board ("ESP-32")         // Wemos ESP-32 or other board, whatever have ESP32 core.
#define Voltage_Resolution (3.3) // 3V3 <- IMPORTANT. Source:
#define ADC_Bit_Resolution (12)  // ESP-32 bit resolution.
#define RatioMQ2CleanAir (9.83)  // RS / R0 = 9.83 ppm

SSD1306Wire display(0x3c, 4, 5); // OLED 地址、 SDA、SCL
BH1750 lightMeter(0x23);         // BH1750地址
Servo myservo;                   // 舵机
DHTesp dht;                      // 温湿度
HardwareSerial MySerial(1);      // 串口1
TempAndHumidity newValues;       // 温湿度

MQUnifiedsensor MQ2(Board, Voltage_Resolution, ADC_Bit_Resolution, MQ_2, "MQ-2"); // MQ2 烟雾

int dhtPin = 10;
char OLED_Buffer[50];
unsigned long lastMsg = 0;
float MQ_2_value = 0;
bool window_status = false;
float lux;
float humi_max = 90.0;
float temp_max = 40.0;
float light_max = 200.0;
float light_mix = 40.0;
float MQ_2_value_max = 60;
int read_key1();
int read_key2();
void blinkLED();
void ASRPRO_Receive(char incoming);
void window_control(bool status);
void MQ_init(void);
void setup()
{
    // put your setup code here, to run once:
    Serial.begin(115200);                     // UART0
    mqtt_init();                              // mqtt初始化
    MySerial.begin(115200, SERIAL_8N1, 1, 0); // UART1
    /*OLED 初始化*/
    display.init();
    display.flipScreenVertically();
    display.setFont(ArialMT_Plain_10);
    // write the buffer to the display
    display.display();

    lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE); // 光照模块初始化
    // Allow allocation of all timers
    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);
    ESP32PWM::allocateTimer(2);
    ESP32PWM::allocateTimer(3);
    myservo.setPeriodHertz(50);   // standard 50 hz servo
    myservo.attach(6, 500, 2500); // 0.5ms-2.5ms
    MQ_init();
    pinMode(LED1, OUTPUT);
    pinMode(LED2, OUTPUT);
    pinMode(BEEP, OUTPUT);
    pinMode(MOTOR, OUTPUT);
    pinMode(KEY1, INPUT_PULLUP);
    pinMode(KEY2, INPUT_PULLUP);
    pinMode(BOMA1, INPUT_PULLUP);
    pinMode(BOMA2, INPUT_PULLUP);
    dht.setup(dhtPin, DHTesp::DHT11); // 温湿度初始化

    digitalWrite(BEEP, HIGH); // 响
    delay(200);
    digitalWrite(BEEP, LOW);
    window_control(false);
}

void loop()
{
    // put your main code here, to run repeatedly:

    newValues = dht.getTempAndHumidity(); // 获取温湿度
    lux = lightMeter.readLightLevel();    // 获取光照强度

    // printf("huimi:%0.2f,temp:%0.2f\r\n",newValues.humidity,newValues.temperature);
    // printf("light: %0.2f lx", lux);

    MQ2.update();
    MQ_2_value = MQ2.readSensor();
    if(MQ_2_value > 3000)
    {
        MQ_2_value = 3000;
    }
    printf("MQ_2_value: %0.2f ppm\r\n", MQ_2_value);

    display.clear();
    display.drawStringf(0, 0, OLED_Buffer, "humi: %0.2f %%", newValues.humidity);
    display.drawStringf(0, 16, OLED_Buffer, "temp: %0.2f 'C", newValues.temperature);
    display.drawStringf(0, 32, OLED_Buffer, "light: %0.2f lx", lux);
    display.drawStringf(0, 48, OLED_Buffer, "gas: %0.2f ppm", MQ_2_value);
    display.display();

    unsigned long now = millis();
    if (now - lastMsg > 1000)
    {
        lastMsg = now;
        mqtt_reconnect();
        mqtt_publish();
    }

    if (MySerial.available() > 0)
    {
        char incoming = MySerial.read();
        // Serial.println("MySerial:");
        Serial.print(incoming);
        ASRPRO_Receive(incoming);
    }

    if (digitalRead(BOMA1) == LOW) // 自动模式
    {
        // Serial.println("BOMA1_ON");
        if (lux > light_max)
        {
            digitalWrite(LED1, LOW);
        }
        if (lux < light_mix)
        {
            digitalWrite(LED1, HIGH);
        }
        if (newValues.humidity > humi_max || newValues.temperature > temp_max)
        {
            digitalWrite(BEEP, HIGH); // 响
            delay(200);
            digitalWrite(BEEP, LOW);
            delay(200);
            digitalWrite(MOTOR, HIGH);
            delay(200);

        }
        else
        {
           digitalWrite(MOTOR, LOW);
        }
        if (MQ_2_value > MQ_2_value_max)
        {
            digitalWrite(BEEP, HIGH); // 响
            delay(200);
            digitalWrite(BEEP, LOW);
            delay(200);
    
            window_control(true);

        }
        else
        {
             window_control(false);
            
        }
    }
    else // 手动模式
    {
        int key1_number = read_key1();
        int key2_number = read_key2();
        if (digitalRead(BOMA2) == LOW) // 控制风扇
        {
            if (key1_number == 1)
            {
                digitalWrite(MOTOR, HIGH);
            }
            else if (key1_number == 0)
            {
                digitalWrite(MOTOR, LOW);
            }
        }
        else
        {
            if (key1_number == 1)
            {
                digitalWrite(LED1, HIGH);
            }
            else if (key1_number == 0)
            {
                digitalWrite(LED1, LOW);
            }
            if (key2_number == 1)
            {
                window_control(true);
            }
            else if (key2_number == 0)
            {
                window_control(false);
            }
        }
    }
    blinkLED();
    sys_delay_ms(500);
}
void blinkLED()
{
    if (digitalRead(LED2) == LOW)
    {
        digitalWrite(LED2, HIGH);
    }
    else
    {
        digitalWrite(LED2, LOW);
    }
}

int read_key1()
{
    static int lastkey1 = 0;
    if (digitalRead(KEY1) == LOW)
    {
        delay(5);
        if (digitalRead(KEY1) == LOW)
        {
            lastkey1 = !lastkey1; // 第一次：1，第二次：0.第三次：1
            return lastkey1;
        }
    }

    return -1;
}
int read_key2()
{
    static int lastkey2 = 0;
    if (digitalRead(KEY2) == LOW)
    {
        delay(5);
        if (digitalRead(KEY2) == LOW)
        {
            lastkey2 = !lastkey2; // 第一次：1，第二次：0.第三次：1
            return lastkey2;
        }
    }
    return -1;
}
void ASRPRO_Receive(char incoming)
{
    if (incoming == '1') // LED
    {
        switch (MySerial.read())
        {
        case '0': // 关
            digitalWrite(LED1, LOW);
            break;
        case '1': // 开
            digitalWrite(LED1, HIGH);
            break;
        }
    }
    if (incoming == '2') // 窗帘
    {
        switch (MySerial.read())
        {
        case '0': // 关
            window_control(false);
            break;
        case '1': // 开
            window_control(true);
            break;
        }
    }
    if (incoming == '3') // 窗帘
    {
        switch (MySerial.read())
        {
        case '0': // 关
            digitalWrite(MOTOR, LOW);
            break;
        case '1': // 开
            digitalWrite(MOTOR, HIGH);
            break;
        }
    }
}

void window_control(bool status)
{
    if (status == true) // 打开
    {
        myservo.write(0);
        window_status = true;
    }
    if (status == false) // 关
    {
        myservo.write(90);
        window_status = false;
    }
}
void MQ_init(void)
{
    MQ2.setRegressionMethod(1); //_PPM =  a*ratio^b
    MQ2.setA(574.25);
    MQ2.setB(-2.222); // Configure the equation to to calculate LPG concentration
    MQ2.init();
    float calcR0 = 0;
    for (int i = 1; i <= 10; i++)
    {
        MQ2.update(); // Update data, the arduino will read the voltage from the analog pin
        calcR0 += MQ2.calibrate(RatioMQ2CleanAir);
        Serial.print(".");
    }
    MQ2.setR0(calcR0 / 10);
    Serial.println("MQ2  done!.");

}