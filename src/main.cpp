#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_INA219.h>

// classes
Adafruit_INA219 SolarPowerSensor;

// const
#define flowOffTimeMax 10 * 60 * 1000
#define flowOnTimeMAX 2 * 60 * 1000
#define TEMP_MAX 50
#define TEMP_MIN 35
#define SERIESRESISTOR 10000
#define TEMPERATURENOMINAL 39
#define THERMISTORNOMINAL 1260
#define BCOEFFICIENT 3950
#define NUMSAMPLES 10
#define MONITORING_DELAY 100
#define BH1750_ADDR 0x23
#define FLOW_CAL_FACTOR 4.5

// pin conf
#define tempSensorControlPin 12
#define tempSolarFrontSensor A0
#define tempSolarBackSensor A3
#define tempWaterInletSensor A2
#define tempCollectorSensor A1
#define pumpPin 11
#define solinoidValvePin 10
#define flowSensorPin 3

// variables
float tempSolarFront = 0;
float tempSolarBack = 0;
float tempWaterInlet = 0;
float tempCollector = 0;
long long solinoidTimeStamp = 0;
bool solinoidOnState = true;
long long monitoringTimeStamp = 0;
byte BH1750_BUFF[2];
float shuntvoltage = 0;
float busvoltage = 0;
float current_mA = 0;
float loadvoltage = 0;
float power_mW = 0;
uint16_t BH1750_val = 0;
volatile uint16_t flowPulseCount = 0;
float flowRate = 0.0;
unsigned int flowMilliLitres = 0;
long long flowOldTime = 0;


// functions
float getTemp(int tempPin);
void BH1750_init();
bool BH1750_read();
void sendData();
void flowPulseCounter();
float calculateFlowRate();

void setup()
{
  // init
  Serial.begin(9600);
  SolarPowerSensor.begin();

  // pin mode select
  pinMode(tempSensorControlPin, OUTPUT);
  pinMode(pumpPin, OUTPUT);
  pinMode(solinoidValvePin, OUTPUT);
  pinMode(flowSensorPin, INPUT);


  // pullup
  digitalWrite(flowSensorPin, HIGH);

  // attach inturrupt 
  attachInterrupt(digitalPinToInterrupt(flowSensorPin),flowPulseCounter,FALLING);
}

void loop()
{
  // if (solinoidOnState && millis() - solinoidTimeStamp >= flowOnTimeMAX)
  // {
  //   solinoidTimeStamp = millis();
  //   solinoidOnState = false;
  // }
  // else if (!solinoidOnState && millis() - solinoidTimeStamp >= flowOffTimeMax)
  // {
  //   solinoidTimeStamp = millis();
  //   solinoidOnState = true;
  // }
  // else
  // {
  //   tempSolarFront = getTemp(tempSolarFrontSensor);
  //   if(tempSolarFront > TEMP_MAX && solinoidOnState){
  //     solinoidOnState = false;
  //     solinoidTimeStamp = millis();
  //   }
  //   else if (tempSolarFront < TEMP_MIN && !solinoidOnState){
  //     solinoidOnState = true;
  //     solinoidTimeStamp = millis();
  //   }
  //   digitalWrite(solinoidValvePin, solinoidOnState);
  // }

  if (millis() - monitoringTimeStamp >= MONITORING_DELAY)
  {
    // read all temp value
    monitoringTimeStamp = millis();
    calculateFlowRate();
    tempSolarFront = getTemp(tempSolarFrontSensor);
    tempSolarBack = getTemp(tempSolarBackSensor);
    tempCollector = getTemp(tempCollectorSensor);
    tempWaterInlet = getTemp(tempWaterInletSensor);
    BH1750_init();
    delay(20);
    if (BH1750_read())
    {
      BH1750_val = (BH1750_BUFF[0] << 8 | BH1750_BUFF[1]);
    }

    shuntvoltage = SolarPowerSensor.getShuntVoltage_mV();
    busvoltage = SolarPowerSensor.getBusVoltage_V();
    current_mA = SolarPowerSensor.getCurrent_mA();
    power_mW = SolarPowerSensor.getPower_mW();
    loadvoltage = busvoltage + (shuntvoltage / 1000);
    sendData();
  }

  delay(1);
}

void sendData()
{
  Serial.print("START;");
  Serial.print("current:");
  Serial.print(current_mA);
  Serial.print(";voltage:");
  Serial.print(busvoltage);
  Serial.print(";power:");
  Serial.print(power_mW);
  Serial.print(";intensity:");
  Serial.print(BH1750_val);
  Serial.print(";solartop:");
  Serial.print(tempSolarFront);
  Serial.print(";solarBack:");
  Serial.print(tempSolarBack);
  Serial.print(";waterInlet");
  Serial.print(tempWaterInlet);
  Serial.print(";collector:");
  Serial.print(tempCollector);
  Serial.print(";flow:");
  Serial.print(flowRate);
  Serial.println(";END");
}

float getTemp(int tempPin)
{
  digitalWrite(tempPin, 1);
  // Serial.println("Start tem reading");
  float avg = 0;

  // read value from sensor
  for (uint8_t i = 0; i < NUMSAMPLES; i++)
  {
    int a = analogRead(tempPin);
    //   Serial.println(a);
    avg += a;
  }
  // cal avg
  avg = avg / NUMSAMPLES;

  // Serial.println(avg);

  // cal thermistor resistance
  avg = 1023 / avg - 1;
  avg = SERIESRESISTOR / avg;

  Serial.println(avg);

  // calculate temp
  float steinhart;
  steinhart = avg / THERMISTORNOMINAL; // (R/Ro)
  steinhart = log(steinhart);          // ln(R/Ro)
  steinhart /= BCOEFFICIENT;
  steinhart += 1.0 / (TEMPERATURENOMINAL + 273.15);
  steinhart = 1.0 / steinhart;
  steinhart -= 273.15;
  Serial.print("Temperature ");
  Serial.print(steinhart);
  Serial.println(" *C");
  digitalWrite(tempPin, 0);
  return steinhart;
}

void BH1750_init()
{
  Wire.beginTransmission(BH1750_ADDR);
  Wire.write(0x10);
  Wire.endTransmission();
}

bool BH1750_read()
{
  int i = 0;
  bool status = false;
  Wire.beginTransmission(BH1750_ADDR);
  Wire.requestFrom(BH1750_ADDR, 2);
  while (Wire.available())
  {
    BH1750_BUFF[i] = Wire.read();
    i++;
  }
  if (i >= 2)
  {
    status = true;
  }

  return status;
}

void flowPulseCounter(){
  flowPulseCount += 1;
}

float calculateFlowRate(){
  detachInterrupt(digitalPinToInterrupt(flowSensorPin));
  long timeDiff = millis() - flowOldTime;
  flowRate = (flowPulseCount * (1000.0/timeDiff)) / FLOW_CAL_FACTOR;
  flowOldTime = millis();
  flowPulseCount = 0;
  attachInterrupt(digitalPinToInterrupt(flowSensorPin), flowPulseCounter, FALLING);
}