/*
 * @Author: Haijia Zhu
 * @Date: 2020-12-17 20:29:47
 * @LastEditTime: 2020-12-18 15:08:47
 * @LastEditors: Please set LastEditors
 * @Description: Coffee bean roaster using Modbus TCP/IP(Board:Goouuu mini S-1)
 * @FilePath: \Coffee Roaster\src\main.cpp
 */
#include <Arduino.h>
#ifdef ESP8266
 #include <ESP8266WiFi.h>
#else
 #include <WiFi.h>
#endif
#include <ModbusIP_ESP8266.h>
#include <max6675.h>
#include <PID_v1.h>

#define LED_I       4
#define LED_R       12
#define LED_G       13
#define LED_B       14
#define SPI_CLK     14  //D14
#define BT_CS       15  //D15 BT 片选
#define ET_CS       5   //D5，ET片选
#define SPI_MISO    12  //D12
#define HEATER_PWM  13  //D16  Heater
#define FAN_PWM     16   //D2   Fan
#define FAN_PWM     2   // LCD screen 片选 暂时没有！  121212

bool state = true;
// MAX6675

MAX6675 BT_t(SPI_CLK, BT_CS, SPI_MISO);
MAX6675 ET_t(SPI_CLK, ET_CS, SPI_MISO);

// Modbus TCP IP
IPAddress remote(192, 168, 30, 109);  // Address of Modbus Slave device
ModbusIP mb;  //ModbusIP object
// Modbus Registers Offsets
const int BT_reg = 1;
const int ET_reg = 2;
const int HEATER_POWER_reg = 3;
const int HEATER_reg = 100;
const int FAN_reg = 101;
const int ON_DMD_coil = 102;

unsigned long ts = 0;
unsigned long ts_pid = 0;
// PID stuff
double Setpoint, Input, Output;
//Specify the links and initial tuning parameters
double Kp=2, Ki=5, Kd=1;
PID heaterPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

void setup() {
  Serial.begin(9600);
  delay(500);
  pinMode(LED_I, OUTPUT);
  Serial.println("");
  Serial.print("Start");  
  WiFi.begin("Can U see me?", "123456790");
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    state = !state;
    digitalWrite(LED_I, state);
  }
  digitalWrite(LED_I, HIGH);
  Serial.println("");
  Serial.println("WiFi connected");  
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  mb.server();
  mb.addHreg(BT_reg, 0);
  mb.addHreg(ET_reg, 0);
  mb.addHreg(HEATER_reg,0);
  mb.addHreg(HEATER_POWER_reg,0);
  mb.addCoil(ON_DMD_coil,0);
  pinMode(LED_R, OUTPUT);
  pinMode(HEATER_PWM,OUTPUT);
  pinMode(FAN_PWM,OUTPUT);
  heaterPID.SetMode(AUTOMATIC);

  analogWriteFreq(5); // change the freqeuency to 5 Hz
}

int tt = 10; //test
double temp_BT = 0;
double temp_ET = 0;
double temp_BT_sum = 0;
unsigned int FAN_pwm =0;  // pwm of fan controller
unsigned int HEATER_pwm =0; // pwm of heater
unsigned int temp_sv = 0; // temperature step point

void loop() {
   mb.task();
     //Read every 1 seconds
   if (millis() > ts + 950) {
      ts = millis();
      //calculate average temperature and
      //send to server
      // mb.Hreg(BT, int(10*temp_BT));
      // mb.Hreg(ET, int(10*ET_t.readCelsius()));
      mb.Hreg(BT_reg, int(10*tt));// testing
      mb.Hreg(ET_reg, int(10*tt));// testing
      mb.Hreg(HEATER_POWER_reg, int(10*tt));// testing
      tt=tt+1;
      state=!state;
      digitalWrite(LED_R, state);
      // read the temperature step point from the master
      temp_sv = mb.Hreg(HEATER_reg);
      FAN_pwm = mb.Hreg(FAN_reg);
      
      
      // FAN control: over writ the FAN pwm for safety
      
      if (temp_sv > 40)
      {
        
        FAN_pwm = 400;
      }
      else if(temp_sv > 10 )
      {
        FAN_pwm = 200;
      }
      analogWrite(FAN_PWM,int(FAN_pwm));
   }
    
   // update PID controll
   if (millis()>ts_pid+190) //about 5.25Hz, 190ms, 60HZ = 16.666ms
   {
     // read temperature
    temp_BT_sum = 0;
    for (int i = 0; i < 10; i++)
    {
      temp_BT_sum = BT_t.readCelsius() + temp_BT_sum;
    }
    temp_BT = temp_BT_sum/10; // mean value of the temperature

     Setpoint = double(temp_sv)/10;
     Input = temp_BT;
     heaterPID.Compute();
     HEATER_pwm = int(Output);
     analogWrite(HEATER_PWM,HEATER_pwm);
   }
}

