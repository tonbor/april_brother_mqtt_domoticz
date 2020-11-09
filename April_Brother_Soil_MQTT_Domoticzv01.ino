/* Soil Moisture Monitor.
  
  Hardware ESP8266 Soil Moisture Probe V2.2 (NodeMcu1.0).
  https://wiki.aprbrother.com/en/ESP_Soil_Moisture_Sensor.html

  Based on Ve2Cuz Real Drouin and Christian Erhardt code

  Fit for MQTT and Domoticz by Rob Gloudi october 2020

  ///////// Pin Assigment ///////

  A0  Input Soil Moisture and Battery
  GPIO4   SDA for tmp112
  GPIO5   SCL for tmp112
  GPIO12  Button S1 (For Homie configuration reset)
  GPIO13  LED
  GPIO14  Clock Output for soil moisture sensor
  GPIO15  SWITCH for measuring Soil Moisture or Battery Voltage

  //////////////////////////////////////////////////////////////////////
*/



#include <ESP8266WiFi.h>
#include <Wire.h>
#include <PubSubClient.h>

WiFiClient espClient;
 PubSubClient client(espClient); 


// Start Your parameters
const char* MY_SSID = "Your_SSID"; 
const char* MY_PWD = "You_Password";
const char* mqtt_server  = "192.168.X.XX";
const int MQTT_Domoticz_IDX = 000;
// End Your parameters

const int PIN_CLK   = D5;
const int PIN_SENSOR  = A0; 
const int PIN_LED   = D7;
const int PIN_SWITCH = D8;
const int PIN_BUTTON = D6;
unsigned long time_now = 0;
// I2C address for temperature sensor
const int TMP_ADDR  = 0x48;

// DEFAULT SETTINGS
// sleep time in microseconds
//#define DEFAULT_DEEP_SLEEP_MINUTES 40  //40 min
#define SLEEP_TIME 3600000000
// use led or not - the led is good for debugging, but not for battery life
const bool DEFAULT_USE_LED = true;
// VCC raw reading @3.0V
const int DEFAULT_VCC_READING_3V = 1024; // 988 3.2volt 2 new battery

// Moisture dry reading @3.0V
const int DEFAULT_MOIST_DRY_READING_AT_3V = 844 ; // 830
// Moisture wet reading @3.0V
const int DEFAULT_MOIST_WET_READING_AT_3V = 703; // 705

// Value range for VCC Readings from 3.0 V to 2.5 V
const int VCC_READING_RANGE = 218 ; //166 1024-806

int batteryCharge = 0;
long rssi;
// if you want to have verbose output and MQTT values published for tuning the soil moisture values, uncomment this:
#define DEBUG


/* 
 * Function: getSendTemperature
 * ----------------------------
 * This function reads the temprature over i2c bus.
 * It then publishes this value via MQTT.
 */
float getSendTemperature() {
  float temperature = 0.0;
  
  // Begin transmission
  Wire.beginTransmission(TMP_ADDR);
  // Select Data Registers
  Wire.write(0X00);
  // End Transmission
  Wire.endTransmission();
 
  // Request 2 bytes , Msb first to get temperature
  Wire.requestFrom(TMP_ADDR, 2);
  // Read temperature as Celsius (the default)
	  if(Wire.available() == 2) {
	    int msb = Wire.read();
	    int lsb = Wire.read();

	    int rawtmp = msb << 8 | lsb;
	    int value = rawtmp >> 4;
	    temperature = value * 0.0625;
	  }
 
    char cstr[6];
    snprintf (cstr, sizeof(cstr), "%f", temperature);
  sendMQTTMessage("Temperature",cstr);
  #ifdef DEBUG
  Serial.print("Function getSendTemperature() ");
  Serial.println(temperature);
  Serial.print("MQTT send");
  Serial.println(cstr);
  #endif
  return(temperature);
}

// Get soil sensor value
int getSendMoisture() {
  // Connect Moisture sensor to the Pin via on PCB switch
  digitalWrite(PIN_SWITCH, HIGH);
  nonBlockingDelay(200);

  int moisture = 0;
  int moist_raw = readSensor();
  int battery_raw = getSendBattery();
  char cstr[16];
  
  
  #ifdef DEBUG
  sprintf(cstr, "%05d", moist_raw);
  sendMQTTMessage("moistureraw",cstr);
 
  
  // Battery Drop Correction to normalize to reading at 3.0V
  Serial.print("Battery charege: ");
  Serial.println(batteryCharge);
   #endif
  moisture = DEFAULT_VCC_READING_3V * moist_raw / battery_raw;
  #ifdef DEBUG
  Serial.print("DEFAULT_VCC_READING_3V * moist_raw / battery_raw; :");
  Serial.println(DEFAULT_VCC_READING_3V);
  Serial.println(moist_raw);
  Serial.println(battery_raw);
  sprintf(cstr, "%05d", moisture);
  sendMQTTMessage("moistureraw2 before map",cstr);
  #endif
  
  // Map the moisture to the min and max reading of the sensor
 moisture = map(moisture, DEFAULT_MOIST_DRY_READING_AT_3V, DEFAULT_MOIST_WET_READING_AT_3V, 0, 100); // Convert to 0 - 100%, 0=Dry, 100=Wet
#ifdef DEBUG
  Serial.print("moisture after map: ");
  
  Serial.println(moisture);
 #endif
  if (moisture > 100) moisture = 100;
  if (moisture <  0) moisture = 0;
  #ifdef DEBUG
 sprintf(cstr, "%d", moisture);
    sendMQTTMessage("moisture",cstr);
     #endif
    return moisture;
}


void setup() {
  Serial.begin(115200);
  Wire.begin();
  pinMode(PIN_CLK, OUTPUT);
  pinMode(PIN_SENSOR, INPUT);
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_SWITCH, OUTPUT);
  pinMode(PIN_BUTTON, INPUT);
  pinMode(D0, WAKEUP_PULLUP);
  digitalWrite(PIN_SWITCH, LOW);
  digitalWrite(PIN_BUTTON, HIGH);


  // device address is specified in datasheet
  Wire.beginTransmission(TMP_ADDR); // transmit to device #44 (0x2c)
  Wire.write(byte(0x01));            // sends instruction byte
  Wire.write(0x60);             // sends potentiometer value byte
  Wire.endTransmission();     // stop transmitting
  
  analogWriteFreq(40000);
  analogWrite(PIN_CLK, 400);
  nonBlockingDelay(500);
  connectWifi();
}

void loop() {
  digitalWrite(PIN_LED, LOW);
  nonBlockingDelay(100);
  digitalWrite(PIN_LED, HIGH);
   float Temperature = 0.0;
   float hum = 0.0;
   int soilmoisturePercent=0;
   char message[32];
   char mqttbuffer[60];
   
  
  Serial.println("Requesting Temperature...");
  
  // Get Domoticz variables and publish them at MQTT
  
   nonBlockingDelay(100);
    soilmoisturePercent=getSendMoisture();
    nonBlockingDelay(200);
    Temperature=getSendTemperature();
      

  sprintf(message, "%2f;%d;%d;", Temperature, soilmoisturePercent, DomoticzHumidityState(soilmoisturePercent));
  #ifdef DEBUG
    Serial.println(message);
  #endif 
  sprintf(mqttbuffer, "{\"idx\":%d,\"nvalue\":0,\"svalue\":\"%s\",\"Battery\":%d,\"RSSI\":%d}",MQTT_Domoticz_IDX,message, batteryCharge, ((rssi + 97) /5 ) +1);
  #ifdef DEBUG
    Serial.println(mqttbuffer);
  #endif      
  // send temperatureto the MQTT topic
  sendMQTTDomoticzMessage(mqttbuffer);
  //Serial.println(" Asleep for %d minutes", SLEEP_TIME);
  nonBlockingDelay(10000);
  
  // sendData(temp, soil_hum);
  // sendMQTTMessage(temp,soil_hum);

  ESP.deepSleep(SLEEP_TIME, WAKE_RF_DEFAULT);
 
  
}

void connectWifi() {
  Serial.print("Connecting to " + *MY_SSID);
  WiFi.begin(MY_SSID, MY_PWD);
	  while (WiFi.status() != WL_CONNECTED) {
	      nonBlockingDelay(1000);
	      Serial.print(".");
	  }
  
  Serial.println("");
  Serial.println("Connected");
  Serial.println("");  
  rssi = WiFi.RSSI();
  Serial.println("Install MQTT broker");
  client.setServer(mqtt_server, 1883);
  // sendMQTTMessage();
}//end connect

void sendMQTTDomoticzMessage( char message[]){
  if (!client.connected()) {
    reconnect();
  }
    const char* mqtt_topic = "domoticz/in"; 
   
  #ifdef DEBUG
    Serial.println(message);
  #endif      
  
 client.publish(mqtt_topic , message);

  nonBlockingDelay(1000);  
}

void sendMQTTMessage(char field[], char message[]){
  if (!client.connected()) {
    reconnect();
  }
  const char* topicSuffix = field;
    char topic[64];
   
  #ifdef DEBUG
  Serial.print("sendMQTTMessage");
  Serial.print(field);
  Serial.println(message);
  
  strcpy(topic, "April Beacon soil Moisture/");
  strcat(topic, topicSuffix);
  
  Serial.print(topic);
  Serial.println(message);
  client.publish(topic , message);
  #endif 


  nonBlockingDelay(1000);  
}


boolean reconnect() {
  Serial.println("Attempting MQTT connection...");
  if (client.connect("ESP8266Client")) {
    Serial.println("connected");
    // client.subscribe("April Beacon soil Moisture/output");

    return client.connected();
  }
  Serial.println("I think connection failed!");
  return 0;
}

void nonBlockingDelay(int waitmillis) {
  time_now = millis();
  while(millis() < time_now + waitmillis) {
    //wait without stopping the cpu
    yield();
  }
}
 
/*
 * Function: getSendBattery
 * ------------------------
 * This function reads the battery voltage multiple times and takes the average of the readings.
 * It then publishes this value via MQTT.
 *  
 * returns: The current battery charge in percent
 */
int getSendBattery() {
  // Connect Battery to the Pin via on PCB switch
  digitalWrite(PIN_SWITCH, LOW);
  nonBlockingDelay(200);

  
  int battery_raw = readSensor();
  char cstr[16];
  #ifdef DEBUG
  sprintf(cstr, "%05d", battery_raw);
  sendMQTTMessage("batteryraw",cstr);
  #endif
  batteryCharge = map(battery_raw, DEFAULT_VCC_READING_3V - VCC_READING_RANGE, DEFAULT_VCC_READING_3V, 0, 100); 
 
 
  if (batteryCharge > 100) batteryCharge = 100;
  if (batteryCharge < 0) batteryCharge = 0;
  #ifdef DEBUG
   sprintf(cstr, "%05d", batteryCharge);
  sendMQTTMessage("batteryCharge",cstr);
  #endif
  return battery_raw;
}
/*
 * Function: readSensor
 * --------------------
 * Reads the sensor connected to pin PIN_SENSOR
 * Since the moisture sensor and the battery voltage are
 * connected to the same input pin, this function can be used
 * for both tasks.
 */
int readSensor() {
  int total = 0;
  int rawVal = 0;
  int ret = 0;
  int sampleCount = 3;

  for(int i = 0; i < sampleCount; i++){
    rawVal = analogRead(PIN_SENSOR);
    
    total += rawVal;
    nonBlockingDelay(50);
  }

  ret = int((float)total / (float)sampleCount);


  return ret;
}
uint8_t DomoticzHumidityState(float h) {
     return (!h) ? 0 : (h < 40) ? 2 : (h > 70) ? 3 : 1;
    }
