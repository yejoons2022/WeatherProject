#include <SoftwareSerial.h>
SoftwareSerial mybt (2, 3);

// 1. 온도, 습도 라이브러리 선언
#include <DHT.h>
#include <DHT_U.h>
#define DHTPIN 4
#define DHTTYPE DHT11
DHT mydht (DHTPIN, DHTTYPE);

// 2. 자외선, 적외선, 가시광선
#include <Wire.h>
#include "Arduino.h"
#include "SI114X.h"
SI114X mylight = SI114X();

// 3. 강우량, 강수량, 강설량 (아날로그 포트 A0 연결)
// 4. 풍속 센서 (아날로그 A3)
// 5. 기압, 고도 센서 연결 i2c, 주소 : 0x77
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
Adafruit_BMP280 mybmp;

//6. 먼지 센서
int dustPin = A2;
int ledPower = 8;
float Vo_value = 0;
float Voltage = 0;

float ds[30];
int dustDensity = 0;
int id = 0;

void setup() {
  Serial.begin (9600);
  mydht.begin();
  mybt.begin(9600);
  Serial.println("Beginning Si1145!");

  while (!mylight.Begin()) {
    Serial.println("Si1145 is not ready!");
    delay(1000);
  }
  Serial.println("Si1145 is ready!");
  pinMode (A0, INPUT);     //강우량
  pinMode (A3, INPUT);     //풍속 측정
  pinMode (A1, INPUT);     //풍향 측정

  Serial.println(F("BMP280 test"));
  if (!mybmp.begin(0x77))
  {
    Serial.println("Could not find a valid BMP280 sensor, check wiring!");
    while(1);
  }
  else
  {
    Serial.println("mybmp check complete!!!!");
  }
  pinMode(ledPower, OUTPUT);
  pinMode(dustPin, INPUT);
  delay(2000);
  
}

void loop() {
  int temp = mydht.readTemperature();     //int : 정수  float : 실수
  int humi = mydht.readHumidity();
  int vl = mylight.ReadVisible();
  int ir = mylight.ReadIR();
  float uv = mylight.ReadUV()/100;
  Serial.print ("temperature : ");
  Serial.println (temp);
  Serial.print ("humidity : ");
  Serial.println (humi);
  Serial.print ("Visible Light : ");
  Serial.println (vl);
  Serial.print ("IR : ");
  Serial.println (ir);
  Serial.print ("UV : ");
  Serial.println (uv);

  mybt.print ("temperature : ");
  mybt.println (temp);
  mybt.print ("humidity : ");
  mybt.println (humi);
  mybt.print ("Visible Light : ");
  mybt.println (vl);
  mybt.print ("IR : ");
  mybt.println (ir);
  mybt.print ("UV : ");
  mybt.println (uv);


  int rainReading = analogRead (A0);
  int range = map (rainReading, 0, 1023, 0, 3);
  switch (range)
  {
    case 0 :
      Serial.println ("not raining");
      mybt.println ("not raining");
      break;
    case 1 :
      Serial.println ("little raining");
      mybt.println ("little raining");
      break;
    case 2 :
      Serial.println ("hard raining");
      mybt.println ("hard raining");
      break;
  }
  int windspread = analogRead(A3);
  float outvoltage = windspread * (5.0 / 1023.0);
  //Serial.print("outvoltage =");
  //Serial.print(outvoltage);
  //Serial.println("V");
  //mybt.print("outvoltage =");
  //mybt.print(outvoltage);
  //mybt.println("V");
  float Level = 6*outvoltage;
  Serial.print("Wind speed is ");
  Serial.print(Level);
  Serial.println("m/s");
  mybt.print("Wind speed is ");
  mybt.print(Level);
  mybt.println("m/s");

  float pressure = mybmp.readPressure()/100;
  float altitude = mybmp.readAltitude(1016.40);
  Serial.print("Pressure : ");
  Serial.print(pressure);
  Serial.println("HPa");
  Serial.print("Altitude : ");
  Serial.print(abs(altitude)*100);
  Serial.println("m");
  mybt.print("Pressure : ");
  mybt.println(pressure);
  mybt.println("HPa");
  mybt.print("Altitude : ");
  mybt.print(abs(altitude)*100);
  mybt.println("m");
  

  for (id = 0 ; id < 30 ; id++)
  {
    if (id == 0);
    {
      dustDensity = 0,0;
    }
    digitalWrite(ledPower, LOW);
    delayMicroseconds(280);
    Vo_value = analogRead(dustPin);
    delayMicroseconds(40);
    digitalWrite(ledPower, HIGH);
    delayMicroseconds(9680);

    Voltage = Vo_value * 5.0 / 1024.0;
    //dustDensity = (Voltage - 0.3) / 0.005;
    ds[id] = (Voltage - 0.3) / 0.005;

    dustDensity +=ds[id];

    if (id == 29)
    {
      dustDensity /= 30;
    }
  }
  Serial.print("micro dust = ");
  Serial.print(dustDensity);
  Serial.println(" ㎍/m3 ");
  Serial.println("");
  mybt.print("micro dust = ");
  mybt.print(dustDensity);
  mybt.println(" ㎍/m3 ");
  mybt.println("");
  
  Serial.println(".....................................................................");
  
  delay (5000);

}
