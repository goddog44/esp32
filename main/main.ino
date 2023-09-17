// Libraries needed
#include <ArduinoJson.h>
#include <Wire.h>   
#include <LiquidCrystal_I2C.h>  
#include <HTTPClient.h>
#include <ESP32Servo.h>
#include <WiFi.h>
#include <Adafruit_AM2320.h>
#include <Adafruit_Sensor.h>
#include "DHT.h" 


// // Constants for sensor and actuator pins
const int gasSensorPin = 12;
const int relayPin1 = 13;
const int relayPin2 = 4;
const int pirPin = 15;

// Joystick pin assignments
const int pinX = 27;    // X-axis analog pin (GPIO 32)
const int pinY = 33;    // Y-axis analog pin (GPIO 33)
const int pinZ = 26;    // Joystick button pin (GPIO 27)

// Incubation options
const char* incubationOptions[] = {"Chicken", "Duck", "Quail", "Turkey", "Goose", "Pheasant", "Emu"};
const int numOptions = sizeof(incubationOptions) / sizeof(incubationOptions[0]);

// State variables
int selectedOption = 0;
bool warmingComplete = false;
bool disinfectionComplete = false;

#define POWER_PIN 17 // ESP32 pin GPIO17 connected to sensor's VCC pin
#define SIGNAL_PIN 35 // ESP32 pin GPIO36 (ADC0) connected to sensor's signal pin

#define POWER_PIN1 5 // ESP32 pin GPIO17 connected to sensor's VCC pin
#define SIGNAL_PIN1 32 // ESP32 pin GPIO36 (ADC0) connected to sensor's signal pi 

// // Objects declarations

LiquidCrystal_I2C lcd(0x27, 16, 2);
Adafruit_AM2320 sensor;
Servo myservo;

const float R0 = 76.63;  // Resistance of the MQ-135 sensor at clean air
const float NH3_CONCENTRATION = 20.0;  // Target ammonia (NH3) concentration in ppm

// char* ssid = "JoMIFI_09904F"; 
// char* password = "jofi09904F";

char* ssid = "Developer"; 
char* password = "#Profiler#25";

const  int waterSensorValue = 566;
const  int waterSensorValue1= 1500;

String INC_SERIAL_NUNBER =  "INC_001";
String  API_BASE_URL =  "http://172.20.10.5:8000/";


// float upperTemperatureThreshold = 100;
// float upperHumidityThreshold = 90;

// float lowerTemperatureThreshold = 37;
// float lowerHumidityThreshold = 39;

// // Traffic light pins
// const int redPin = 4;
// const int yellowPin = 5;
// const int greenPin = 6;

// // Traffic light status
// enum TrafficLightStatus {
//   NORMAL,
//   ABNORMAL,
//   OBJECT_DETECTED
// };

// TrafficLightStatus currentStatus = NORMAL;

// Connect to Wi-Fi

void send_g_t_h(float gas, float temperature, float humidity){
  HTTPClient http;
  String path = API_BASE_URL + "incubation_state/" + INC_SERIAL_NUNBER + "/";
  http.begin(path);
  http.addHeader("Content-type","application/json");

  StaticJsonDocument<200> jsonDoc;
  JsonObject root = jsonDoc.to<JsonObject>();

  root["temparature"] = temperature;
  root["humidity"] = humidity;
  root["gas"] = gas;

  String jsonString;
  serializeJson(root, jsonString);

  int result = http.POST(jsonString);
  Serial.print("HTTP RESPONSE  :  ");
  Serial.println(result);

   http.end();

}



void send_movement_notif(){

//  http://127.0.0.1:8000/send_notification_movement/INC_001/

  HTTPClient http;
  String path = API_BASE_URL + "send_notification_movement/" + INC_SERIAL_NUNBER + "/";
  http.begin(path);
  http.addHeader("Content-type","application/json");

  

  int result = http.POST({});
  Serial.print("HTTP RESPONSE  :  ");
  Serial.println(result);

   http.end();

}

void connectToWiFi() {
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    Serial.println("Connecting to WiFi..."); 
    
    String message = "connecting to WiFi...";
    int messageLength = message.length();
  
  // Scroll the message from right to left
  for (int i = 0;i <= messageLength; i++) {
    lcd.clear();
    lcd.setCursor(0, 0);
    
    if (i < messageLength) {
      lcd.print(message.substring(i));
    } else {
      lcd.print(message.substring(0));
    }
    
    delay(500);
  }
}
  Serial.println();
  Serial.print("Connected to Wi-Fi: ");
  lcd.clear();
  lcd.print("Wi-Fi Connected : ");
  Serial.println(WiFi.SSID());
  lcd.setCursor(0, 1);
  lcd.print(WiFi.SSID());
  delay(1000);
  lcd.clear();
  Serial.print("IP address: ");
  lcd.print("IP address: ");
  Serial.println(WiFi.localIP());
  lcd.setCursor(0, 1);
  lcd.print(WiFi.localIP());
  delay(1000);
}

int readWaterSensorValue() {
  int value = 0; // variable to store the sensor value

  digitalWrite(POWER_PIN, HIGH);  // turn the sensor ON
  delay(10);                      // wait 10 milliseconds
  value = analogRead(SIGNAL_PIN); // read the analog value from the sensor
  digitalWrite(POWER_PIN, LOW);   // turn the sensor OFF

  return value;
}
int readWaterSensorValue1() {
  int value1 = 0; // variable to store the sensor value

  digitalWrite(POWER_PIN1, HIGH);  // turn the sensor ON
  delay(10);                      // wait 10 milliseconds
  value1 = analogRead(SIGNAL_PIN1); // read the analog value from the sensor
  digitalWrite(POWER_PIN1, LOW);   // turn the sensor OFF

  return value1;
}

void getHumandTemp(float& humidity, float&temperature) {
  temperature = sensor.readTemperature();
  humidity = sensor.readHumidity();

  // Check if the readings are valid
  if (!isnan(temperature) && !isnan(humidity)) {
  } else {
    Serial.println("Failed to read data from AM2320 sensor");
  }

  delay(2000); // Wait for 2 seconds before reading again
}
// Function to control fan (relayPin1) based on temperature
void checkTemperature(float temperature, float humidity) {
  if (temperature > 35 || humidity > 80) {
    digitalWrite(relayPin1, HIGH); 
    Serial.println("Fans ON");
    delay(1000);    // Set relay pin to HIGH
  } else {
    digitalWrite(relayPin1, LOW);  
    Serial.println("Fans OFF");  
    delay(1000);   // Set relay pin to LOW
  }
}
void checkHumidity(float humidity, float upperHumidityThreshold) {
  if (humidity > 60) {
    digitalWrite(relayPin1, HIGH); 
    Serial.println("Fan ON");    // Set relay pin to HIGH
  } else {
    digitalWrite(relayPin1, LOW);  
    Serial.println("Fan OFF");    
  }
}


// // Function to update the traffic light based on the current status
// void updateTrafficLight() {
//   switch (currentStatus) {
//     case NORMAL:
//       digitalWrite(redPin, LOW);
//       digitalWrite(yellowPin, LOW);
//       digitalWrite(greenPin, HIGH);
//       break;
//     case ABNORMAL:
//       digitalWrite(redPin, LOW);
//       digitalWrite(yellowPin, HIGH);
//       digitalWrite(greenPin, LOW);
//       // Beep or perform other abnormality indication actions
//       break;
//     case OBJECT_DETECTED:
//       digitalWrite(redPin, HIGH);
//       digitalWrite(yellowPin, LOW);
//       digitalWrite(greenPin, LOW);
//       break;
//   }
// }

// // ************************************Function to rotate the servo after 12 hours
// void rotateServo() {
//   myservo.attach(25);
//   lcd.clear();
//   lcd.setCursor(0, 0);
//   lcd.print("Inclin : Left");
//   myservo.write(90);
//   lcd.clear();
//   lcd.setCursor(0, 0);
//   lcd.print("Inclin : Right");
//   delay(1000);
//   myservo.detach();
// }

// Function to make an HTTP request
void httpRequest() {
  const String url = "http://172.20.10.2:5000";
  const String route = "/all";
  const String urlss = url + route;

  HTTPClient http;
  http.begin(urlss);
  http.addHeader("Content-Type", "application/json");

  const size_t capacity = JSON_OBJECT_SIZE(1);
  StaticJsonDocument<capacity> jsonDoc;

  String jsonString;
  serializeJson(jsonDoc, jsonString);
  int httpResponseCode = http.POST(jsonString);

  if (httpResponseCode > 0) {
    Serial.print("HTTP response code: ");
    Serial.println(httpResponseCode);
  } else {
    Serial.print("Error sending HTTP request. Error code: ");
    Serial.println(httpResponseCode);
  }

  http.end();
  delay(1000);
}

void detectAmmoniaGas() {
  // Read the analog value from the MQ-135 sensor
  int sensorValue = analogRead(gasSensorPin);

  // Convert the analog value to resistance
  float sensorResistance = ((1023.0 / sensorValue) - 1.0) * 10000.0;

  // Calculate the gas concentration using the sensor resistance and calibration data
  float gasConcentration = pow(10, ((log10(sensorResistance / R0) - 0.326) / -0.744));

  // Print the gas concentration in ppm
  Serial.print("Ammonia Concentration: ");
  Serial.print(gasConcentration);
  Serial.println(" ppm");
  lcd.clear();
  lcd.print("Ammonia conc:");
  lcd.print(sensorValue);
  lcd.println("ppm");
  
  // Check if the gas concentration exceeds the target concentration
  if (gasConcentration > NH3_CONCENTRATION) {
   while(gasConcentration > NH3_CONCENTRATION){
     digitalWrite(relayPin1, HIGH);
   }
    Serial.println("Ammonia detected!");
    lcd.clear();
    lcd.println("Ammonia detected!");
  }

  delay(500);  // Delay between readings
}
void Joystick(){
   //Read analog values from the joystick
    // c:
  int xValue = analogRead(pinX);
  int yValue = analogRead(pinY);
  int zValue = digitalRead(pinZ);

//   Serial.print("Valeur Y ");
//   Serial.println(yValue);

//   Serial.print("Valeur X ");
//   Serial.println(  xValue);

//   Serial.print("Valeur Z ");
//   Serial.println(  zValue);
// delay(3000);
// goto c;

  // Move down the incubation options
  b:
   xValue = analogRead(pinX);
   yValue = analogRead(pinY);
   zValue = digitalRead(pinZ);
  if (xValue < 4000) {
    selectedOption = (selectedOption + 1) % numOptions;
    lcd.clear();
     lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Incubation Type");
    Serial.println("Incubation Type"); 
    lcd.setCursor(4, 4);
    lcd.print(incubationOptions[selectedOption]);
    Serial.println(incubationOptions[selectedOption]);
    delay(1400);
  
  }
    Serial.print("Valeur Z ");
    Serial.println(  zValue);
    delay(3000);
  goto b;
  // Start incubation on button press
  if (zValue == LOW) {
    if (!warmingComplete) {
      warmingComplete = true;
      lcd.clear();
      lcd.print("Warming Process..");
      Serial.println("Warming Process..");
      delay(6000);
      lcd.clear();
      lcd.print("Warming Completed!");
      Serial.println("Warming Completed!");
      delay(2000);

      // Perform warming process and reach threshold
      // ...
    } else if (!disinfectionComplete) {
      disinfectionComplete = true;
      lcd.clear();
      lcd.print("Disinfection Process...");
      Serial.println("Disinfection Process...");
      delay(10000);  // 10 minutes delay for disinfection
      lcd.clear();
      lcd.print("Disinfection Completed");
      Serial.println("Disinfection Cpmpleted");
      delay(1000);

    } 

      zValue = digitalRead(pinZ);

      while(zValue != 0){
        zValue = digitalRead(pinZ);
        lcd.clear();
        lcd.print("Start now?");
        Serial.println("Start now?");
        delay(1000);
    }

      // Start incubation process
      if(zValue == LOW){
        lcd.clear();
        lcd.print("Incubation Start!");
        Serial.println("Incubation Start!");
        delay(10000);
      // Perform the rest of the incubation process
      // ...
      }
  }
}
void setup() {
  Serial.begin(115200);
  Wire.begin();
  sensor.begin();
  lcd.begin(16, 2);
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Essono & Tiomi");
  lcd.setCursor(1, 3);
  lcd.print("Smart Egg Incub");
  delay(1000);
  connectToWiFi();
  //***********************************************************************************ANALOG****************************************************************
  pinMode(pinZ, INPUT_PULLUP);
  digitalWrite(pinZ, HIGH);
  // Initialize the LCD module
   
  Joystick();
  delay(5000);
//**********************************************************************************************************************************************************************************
  pinMode(relayPin1, OUTPUT);    // pinMode(relayPin2, OUTPUT);
  pinMode(pirPin, INPUT);
  pinMode(POWER_PIN, OUTPUT);   // configure pin as an OUTPUT
  pinMode(POWER_PIN1, OUTPUT);   // configure pin as an OUTPUT
  pinMode(SIGNAL_PIN, INPUT); 
  pinMode(SIGNAL_PIN1, INPUT); 

}

void motionDtection(){
   int val = digitalRead(pirPin);
  if (val == HIGH) {
    send_movement_notif();
    Serial.println("Motion detected!");
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Motion Detected");
    }
  else {

    Serial.println("No Motion detected!");
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("No Motion");
    lcd.setCursor(0, 1);
    lcd.print(" Detected");
    }
    
delay(10);
}

void loop() {
 
  // controlFanT(temperature);
  // controlFanH(humidity);
  // updateTrafficLight();
  // httpRequest();

  float temperature = sensor.readTemperature();
  float humidity = sensor.readHumidity();
  getHumandTemp(humidity, temperature);

    Serial.print("Temperature: ");
    Serial.print(temperature);
    Serial.println(" °C");

    Serial.print("Humidity: ");
    Serial.print(humidity);
    Serial.println(" %");

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Temp: ");
  lcd.print(temperature);
  lcd.print(" 'C");
  lcd.setCursor(0, 1);
  lcd.print("Humidity: ");
  lcd.print(humidity);
  lcd.print(" %");

  checkTemperature(temperature, humidity);
    delay(1000);
  
  detectAmmoniaGas();
  delay(1000);

    // rotateServo();
  motionDtection();
  delay(1000);

  // Your main code here
  int waterSensor = readWaterSensorValue();
  int waterSensor1 = readWaterSensorValue1();


  waterSensor = (waterSensor * 100) / waterSensorValue;

  waterSensor1 = (waterSensor1 * 100) / waterSensorValue1;

  Serial.print("The water sensor value: ");
  Serial.println(waterSensor);
  Serial.print("The water sensor value1: ");
  Serial.println(waterSensor1);

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Water Lev1 : "); 
  lcd.println(waterSensor);
  lcd.setCursor(0, 1);
  lcd.print("Water Lev2 : "); 
  lcd.println(waterSensor1);
  lcd.println(" %");
  delay(1000);


  // while (digitalRead(push1) == HIGH && pos < 180) {
  //   pos++;
  //   myservo.write(pos);
  //   delay(15);
  // }
  // while (digitalRead(push2) == HIGH && pos > 0) {
  //   pos--;
  //   myservo.write(pos);
  //   delay(15);
  // }
  // Serial.println(push2);

}
