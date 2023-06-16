#include "arduino_secrets.h"
#include <Arduino.h>
#include "DFRobot_EnvironmentalSensor.h"
#if defined(ARDUINO_AVR_UNO)||defined(ESP8266)
#include <SoftwareSerial.h>
#endif
#define MODESWITCH        /*UART:*/0 /*I2C: 1*/
#if MODESWITCH
#if defined(ARDUINO_AVR_UNO)||defined(ESP8266)
  SoftwareSerial mySerial(/*rx =*/4, /*tx =*/5);
  DFRobot_EnvironmentalSensor environment(/*addr =*/SEN050X_DEFAULT_DEVICE_ADDRESS, /*s =*/&mySerial);//Create an object for broadcast address, which can configure all devices on the bus in batches  
#else
  DFRobot_EnvironmentalSensor environment(/*addr =*/SEN050X_DEFAULT_DEVICE_ADDRESS, /*s =*/&Serial1); //Create an object for broadcast address, which can configure all devices on the bus in batches
#endif
#else
DFRobot_EnvironmentalSensor environment(/*addr = */SEN050X_DEFAULT_DEVICE_ADDRESS, /*pWire = */&Wire);
#endif

#include <EloquentTinyML.h>
#include <eloquent_tinyml/tensorflow.h>
#include "model.h"

#define NUMBER_OF_INPUTS 3
#define NUMBER_OF_OUTPUTS 2
#define TENSOR_ARENA_SIZE 5*1024
Eloquent::TinyML::TensorFlow::TensorFlow<NUMBER_OF_INPUTS, NUMBER_OF_OUTPUTS, TENSOR_ARENA_SIZE>ml;

// Means and standard deviations from our dataset curation
static const float means[] = {72.4314, 1014.0769, 18.5521};
static const float std_devs[] = {14.6934, 2.359, 3.7788};

//Air quality sensor
#include <DFRobot_SGP40.h>
DFRobot_SGP40    mySgp40;


#include "thingProperties.h"

const int soil_s=A0;
const int myswitch= D7;
bool thisState ;
bool lastMySwitchState;
const int relay = D9 ;
bool automode;

void setup() {
  //pinMode(myLED, OUTPUT);
  #if MODESWITCH
  //Initialize MCU communication serial port
#if defined(ARDUINO_AVR_UNO)||defined(ESP8266)
  mySerial.begin(9600);
#elif defined(ESP32)
  Serial1.begin(9600, SERIAL_8N1, /*rx =*/D3, /*tx =*/D2);
#else
  Serial1.begin(9600);
#endif
#endif
  Serial.begin(115200);
  Serial.println();
  Serial.println("Configuring access point...");

  

    while(environment.begin() != 0){
    Serial.println(" Sensor initialize failed!!");
    delay(1000);
  }
   Serial.println("sgp40 is starting, the reading can be taken after 10 seconds...");
  while(mySgp40.begin(/*duration = */10000) !=true){
    Serial.println("failed to initialize Air quality sensor, please  connection is fine");
    delay(1000);}
    
  Serial.println(" Sensor  initialize success!!");
  pinMode(myswitch,INPUT);
  pinMode(relay,OUTPUT);
  pinMode(soil_s,INPUT);


  ml.begin(model);

  // Defined in thingProperties.h
  initProperties();

  // Connect to Arduino IoT Cloud
  ArduinoCloud.begin(ArduinoIoTPreferredConnection);
  
  /*
     The following function allows you to obtain more information
     related to the state of network and IoT Cloud connection and errors
     the higher number the more granular information you’ll get.
     The default is 0 (only errors).
     Maximum is 4
 */
  setDebugMessageLevel(2);
  ArduinoCloud.printDebugInfo();
}

void loop() {
  ArduinoCloud.update();
  // Your code here 
  float temperature = environment.getTemperature(TEMP_C);
  float humidity = environment.getHumidity();
  float pressure = environment.getAtmospherePressure(HPA);
  uint16_t index = mySgp40.getVoclndex();
  //to cloud
   airQ=index;
   temp=temperature;
   hum=humidity;
  //irrigation
  irrigation_setting(automode);
   
  // Perform standardization on each reading
  // Use the values from means[] and std_devs[]
  temperature = (temperature - means[2]) / std_devs[2];
  humidity = (humidity - means[0]) / std_devs[0];
  pressure = (pressure - means[1]) / std_devs[1];

  float input[NUMBER_OF_INPUTS] = {humidity,pressure,temperature};
  float output[NUMBER_OF_OUTPUTS] = {0,0};
  ml.predict(input,output);
  float pred =ml.getScoreAt(output[0]);
  //to cloud
  rain=pred*10;


  delay(500);
}



/*
  Since SoilM is READ_WRITE variable, onSoilMChange() is
  executed every time a new value is received from IoT Cloud.
*/

/*
  Since SoilM is READ_WRITE variable, onSoilMChange() is
  executed every time a new value is received from IoT Cloud.
*/
void onSoilMChange()  {
  // Add your code here to act upon SoilM change
  if (soilM == 1) {
    automode=1;
    }
    else{
      automode=0;
    }
}
void irrigation_setting(bool mode)
{ if(mode ==1){//automatic
  float soilS=analogRead(A0);
  float soilC =100*(1-(soilS/4095));// calculating soil water %
  if (soilC < 30){
    digitalWrite(relay,HIGH);
  }
  else if (soilC>30)
  {
    digitalWrite(relay,LOW);
  }
  }
  if (mode== 0)
  {
     thisState = digitalRead(myswitch);
  if (thisState != lastMySwitchState)
  {  
    //update the switch state
    lastMySwitchState = thisState;  

    //"HIGH condition code"
    //switch goes from LOW to HIGH
    if(thisState == HIGH)        
    {
      //relay on pin 9 is Push ON, Push OFF
      digitalWrite(relay,!digitalRead(relay)); 
    }
  }
  }
  
}

