//  Project:  RIOT2 - Remote IOT Node for monitoring Weather - Temperature, Humidity, Barometric Pressure
//  Author:   Geofrey Cardoza
//  Baseline: August 31st, 2016
//  Revision: September 20th, 2016
//
//  Hardware Configuration:
//    AdaFruit Feather Huzzah with ESP8266 Micro-controller
//      - WiFi & MQTT messaging interface
//    BME280 for Temperature, Humidity and Pressure (Primary)
//    DHT22 for Temperature and Pressure (Secondary)
//    BMP180 for Pressure (Secondary)
//    Analog (0-1V) Moisture Sensor
//    Alarm - digital Input Sensor
//    Digital 1 input sensor

  const char* Code_Version = " 1.0s";

// ***** Include header files *****
  #include <PubSubClient.h>         // Library for MQTT Pub/Sub functions
  #include <ESP8266WiFi.h>          // Library for ESP8266 WiFi microcontroller
  #include "DHT.h"                  // Library for DHT22 Temperature and Humidity sensor
  #include <Wire.h>                 // Library for I2C Communication
  #include <SFE_BMP180.h>           // Library for BMP180 Pressure and Temperature sensor
  #include <Adafruit_Sensor.h>
  #include <Adafruit_BME280.h>
  
// ***** WiFi & Server Info *** SET FOR EACH IMPLEMENTATION *****
  #define MQTT_Server "192.168.0.120"
  const char* ssid = "Excal-AS-RC";
  const char* WiFi_Password = "6677889900";
  const char* Node_Type = "RIOT2";
  char Node_Id[30];

// ***** Declare global variables for the RIOT2 Node
  int Update_Interval = 60000;            // set default Update interval to 60 sec. (can be changed)
  int sleepDuration = 2;                  // time the cpu goes into deep sleep after every loop
  float Temperature, Humidity, Pressure, Altitude, Analog, Alarm, Digital_1;
  unsigned long Update_Sequence = 0;      // Update sequence number to base
  char Sensor_Data[150];                  // Buffer to hold formatted sensor data payload
  
// ***** Define variables to track how long the program has been running *****
  long Current_Time;
  long Last_Publish_Time = 0;

// ***** MQTT pub/sub service info *****
  const char* Sensor_Topic = "/RioT/SensorData";
  const char* Control_Topic = "/Control/RioT";
  const char* MQTT_Id = "biot";
  const char* MQTT_Pw = "excaliber";
  WiFiClient espClient;
  PubSubClient client(espClient);
  char Control_Command[10];        // Inbound subscribed control message command
  int  Control_Data;               // Inbound subscribed control message data
  
// ***** Set BME280 Temperature, Humidity and Pressure Variables *****
  #define SEALEVELPRESSURE_HPA (1013.25)
  int Home_Altitude = 317;        // Altitude in meters to compensate for Barometric Pressure 
  Adafruit_BME280 bme;            // I2C
  boolean BME280_Present = true;  // set the default to the board being present
  
// ***** Set DHT22 Temp and Pressure Variables *****
  #define DHT_Type DHT22          // DHT 22  (AM2302), AM2321
  #define DHT22_Pin 2             // The digital IO pin the DHT22 is connected to
  DHT dht(DHT22_Pin, DHT_Type);
  
// ***** Set BMP180 Pressure Sensor Variables *****
  SFE_BMP180 myBMP180;
 
// ***** Set Analog and Digital Input Variables *****
  #define Analog_Pin  0          // Analog input pin
  #define Alarm_Pin   12         // Alarm input digital pin
  #define Digital1_Pin 13         // Digital input 1 pin


// ********** INITIALIZE ALL COMPONENTS OF THE SYSTEM **********
void setup(void)
{
  // ***** Start the serial port for debugging vi Arduino Serial Monitor *****
  Serial.begin(115200);
  delay(1000); //Pause to allow serial port to initialize
  
  Serial.print("\n***** STARTING RIOT2 - Code Version: ");
  Serial.print(Code_Version);
  Serial.println(" *****");
  Serial.println("\n***** ENTERING SETUP MODE *****");

  // ***** Start WiFi Communication Subsystem *****
  Serial.print("-> Connecting to WiFi Network\n");
  setup_wifi();

  // ***** Configure & Start MQTT Messaging service *****
  Serial.println("-> MQTT: Configuring Messaging Service");
  client.setServer(MQTT_Server, 1883); // Connect to MQTT Server
  Serial.print("  -> Server Address: ");
  Serial.println(MQTT_Server);
  client.setCallback(callback);        // Set the callback function when subscribed message arrives 
  client.subscribe(Control_Topic);     // Subscribe to Control Topic
        
  // ***** Check for BME280 Presence and if found Initialize it *****
  Serial.print("-> BME280: Starting Temperature, Humidity and Pressure Sensor\n");
  if (!bme.begin())
  {
    BME280_Present = false;
    Serial.println("  -> BME280: Could not find sensor. Using DHT22 & BMP180.");
  }

  if(!BME280_Present)
  {
    // ***** Initialize DHT22 Temperature and Humidity sensor *****
    Serial.print("-> DHT22: Starting Temperature & Humidity Sensor\n");
    dht.begin();

    // ***** Initialize BMP180 Pressure sensor *****
    Serial.print(F("-> BMP180: Starting Pressure and Temperature Sensor "));
    myBMP180.begin();
    Serial.print(F("\n"));    
  }

  // ***** Configure Onboard LED (Pin 2) and set to Off *****
  Serial.print("-> LED: Configuring Onboard LED GPIO 0 and set to off\n");
  pinMode(BUILTIN_LED, OUTPUT);
  digitalWrite(BUILTIN_LED, HIGH);  // Turn the LED off by making the voltage HIGH
  
  // ***** Configure Alarm and Digital Input Pins *****
  Serial.print("-> Analog & Digital Input: Configuring A/D and GPIO ports\n");
  pinMode(Alarm_Pin, INPUT_PULLUP);
  pinMode(Digital1_Pin, INPUT_PULLUP);
    
  //Wait 2 seconds to have sensors stabilize
  delay(2000);
  
  //Force a Sensor Reading on the first loop
  Last_Publish_Time = millis() - Update_Interval;
}


// ********** MAIN PROGRAM LOOP **********
void loop()
{
  // ***** Wait Update Interval prior to Publishing Sensor Data *****
  Current_Time = millis();    // get current program run=time in ms
  if (Current_Time-Last_Publish_Time > Update_Interval)
  {
    Serial.println("\n***** PREPARING TO READ AND PUBLISH SENSOR DATA *****");
    Last_Publish_Time = Current_Time;   // Reste last publish time

    // ***** Read Sensors and Store Data in Global variables *****
    if(BME280_Present)
    {
      read_BME280();
    }
    else
    {
      read_DHT22();
      read_BMP180();     
    }
    read_Analog_Digital();    

    // ***** Format Sensor Data Payload and send to BIOT Base
    Publish_Sensor_Data();

  }
  // ***** Check for and Process Subscribed Messages *****
  client.loop();
}

// *************** Sub-Routines *********************

//  ***** Read the Temperature, Humidity and Pressure from the BME280 over I2C *****
int read_BME280()
{
  float P, Correction_Factor;        // Var to adjust altitude correction factor for pressure
  Serial.println("-> BME280: Reading Temperature, Humidity, and Pressure");

  Temperature = bme.readTemperature();
  P = bme.readPressure()/(1000.0);                  // Convert to kPa
  Humidity = bme.readHumidity();
  Altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);
 
  // Correct Pressure based on current altitude read bythe BME280
  Correction_Factor = (760-(Home_Altitude*3.281*0.026))/760;  // Corrects Pressure to Sea Level
  Pressure = P/Correction_Factor;
  
  Serial.print("  -> Temp = ");
  Serial.print(Temperature);
  Serial.print(" *C, Hum = ");
  Serial.print(Humidity);
  Serial.print(" %, Pres = ");
  Serial.print(Pressure);
  Serial.print(" kPa, Alt = ");
  Serial.print(Altitude);
  Serial.print(" m, Correction Factor = ");
  Serial.println(Correction_Factor);
  return(0);
}

//  ***** Read the Temperature and Humidity from the DHT22 sensor over a serial digital I/O port *****
int read_DHT22()
{
  Serial.println("-> DHT22: Reading Temperature and Humidity");
  
  // Note Reading temperature or humidity takes between .25 - 2 seconds
  Humidity = dht.readHumidity();        // Relative Humidity in %
  Temperature = dht.readTemperature();  // Temp in Celcius, for Fahrenheit use dht.readTemperature(true)

  // Check if DHT22 reads failed and exit
  if (isnan(Humidity) || isnan(Temperature))  // isnan = is not a number
  {
    Serial.println("  -> Failed to read from DHT22 sensor!");
    //Set the failed read values for the readings
    Humidity = 999;
    Temperature = 999;
    return(-1);
  }

  Serial.println("  -> DHT22 Read successful");
  Serial.print("  -> Temperature: ");
  Serial.print(Temperature);
  Serial.print(" *C, ");
  Serial.print("Humidity: ");
  Serial.print(Humidity);
  Serial.println(" %, ");
  return(0);
}

//  ***** Read the Temperature and Pressure from the BMP180 sensor over I2C *****
int read_BMP180()
{
  char status;
  double T,P,p0,a;

  Serial.println("-> BMP180: Reading Temperature and Pressure");
  
  status = myBMP180.startTemperature();
  if (status != 0)
  {
    delay(status);
    status = myBMP180.getTemperature(T);
    if (status != 0)
    { 
      status = myBMP180.startPressure(3);
      if (status != 0)
      {
        delay(status);
        status = myBMP180.getPressure(P,T);
        if (status != 0)
        {
          //Correct pressure to sea level & convert to kPa
          Pressure = myBMP180.sealevel(P,Home_Altitude)/10;

          Serial.print("Barometric Pressure: ");
          Serial.print(Pressure);
          Serial.println(" kPa");
          return(0);
        }
        else
        {
          Serial.print(F("  -> BMP180: ERROR Starting Pressure Measurement"));
          Pressure = 999;
          return(-4);
        }
      }
      else
      {
        Serial.print(F("  -> BMP180: ERROR Starting Pressure Measurement"));
        Pressure = 999;
        return(-3);
      }
    }
    else
    { 
      Serial.print(F("  -> BMP180: ERROR Reading Temperature"));
      Pressure = 999;
      return(-2);
    }
  }
  else
  {  
    Serial.println(F("  -> BMP180: ERROR Starting Temperature Measurement"));
    Pressure = 999;
    return(-1);
  }
}

// ***** Get Analog and Digital Input Readings from microcontroller and store in Global Vars *****
void read_Analog_Digital()
{
  Serial.println("-> Reading Analog & Digital Inputs");

  Analog = analogRead(Analog_Pin);   // A?D reading in 10 bit resolution. 1V = 1023

  // Note that the GPIO reads are reversed due to Open Collector Pull-ups
  //   a High means not activated, and a Low means activated
  if(digitalRead(Alarm_Pin) == HIGH) Alarm = 0;
  else Alarm = 1;
  
  if(digitalRead(Digital1_Pin) == HIGH) Digital_1 = 0;
  else Digital_1 = 1;
  
  Serial.print("  -> Analog: ");
  Serial.print(Analog);
  Serial.print(" Alarm: ");
  Serial.print(Alarm);
  Serial.print(" Digital1: ");
  Serial.println(Digital_1);
}

// ***** Connect to the WiFi Network and establish Node Name *****
void setup_wifi()
{
  uint8_t mac[6];
    
  delay(10);
  // Connect to the WiFi network
  Serial.print("  -> Connecting to ");
  Serial.print(ssid);
  Serial.print(" ");
 
  WiFi.begin(ssid, WiFi_Password);

  while (WiFi.status() != WL_CONNECTED) 
  {
    delay(1000);
    Serial.print(".");
  }
  Serial.println("Connected");
  
  // Create Node ID = Node_Type-MAC Address
  WiFi.macAddress(mac);
  sprintf(Node_Id, "%5s-%02x:%02x:%02x:%02x:%02x:%02x",Node_Type, mac[0],mac[1],mac[2],mac[3],mac[4],mac[5]);

  Serial.print("  -> Node IP: ");
  Serial.println(WiFi.localIP());
  Serial.print("  -> Node ID: ");
  Serial.println(Node_Id);
}

// ***** Process Subscribed message received from BIOT *****
void callback(char* topic, byte* payload, unsigned int length)
{  
  char data_b[100];
  int i, j;
  
  Serial.println("\n***** Subscribed Topic Message Arrived *****");
  Serial.print ("-> Topic: ");
  Serial.print(topic);
  Serial.print(", Message Length: ");
  Serial.println(length);
  Serial.print("-> Message: ");
  for (int i = 0; i < length; i++) Serial.print((char)payload[i]);
  Serial.println();

  // Flash LED for 0.5 seconds to indicate message arrival
  digitalWrite(BUILTIN_LED, LOW);   // Note LOW turns LED on
  delay(500);
  digitalWrite(BUILTIN_LED, HIGH);  // Turn the LED off

  // ***** Process Inbound Command *****
  Serial.print("-> Processing Inbound Control Command");
  if(length ==7)
  {
    sprintf(Control_Command, "%c%c%c\0", payload[0], payload[1], payload[2]);
    sprintf(data_b, "%c%c%c%c\0", payload[3], payload[4], payload[5], payload[6]);

    Control_Data = atoi(data_b);
    Serial.print("  -> Control Command = ");
    Serial.print(Control_Command);
    Serial.print(" , Control Data = ");
    Serial.println(Control_Data);

    // ** 1. Change Sensor Data Update Interval **
    if(!strcmp(Control_Command, "UF:") && Control_Data >= 5)
    {
      Serial.print("  -> Changing Sersor Data Update Interval to ");
      Serial.print(Control_Data);
      Serial.println(" seconds");
      Update_Interval = Control_Data *1000;    // Set Sensor Data Update Interval
      return;
    }

    // ** 2. Force Node to Publish Sensor Data NOW **
    if(!strcmp(Control_Command, "UN:") && Control_Data == 9999)
    {
      Serial.print("  -> Forcing Node to Publish Sensor Data NOW");
      Last_Publish_Time = millis() - Update_Interval;
      return;
    }
    
    // ** 3. Force Node to Reset the Update Sequence number to 0 **
    if(!strcmp(Control_Command, "RS:") && Control_Data == 9999)
    {
      Serial.print("  -> Forcing Node to Reset Sequence Number to 0");
      Update_Sequence = 0;
      return;
    }

  }
  else Serial.println("  -> Invalid command received");
}

// ***** Reconnect to MQTT service and subscribe to Control Topic *****
void reconnect()
{
  // Loop until we're reconnected
  while (!client.connected())
  {
    Serial.print("\n  -> Attempting MQTT connection...");
    
    // Attempt to connect to the MQTT server
    if (client.connect(Node_Id, MQTT_Id, MQTT_Pw)) 
    {
      Serial.println("connected");
      
      // Resubscribe to the MQTT Configuration topic
      Serial.print("  -> Subscribing to MQTT Control Topic service\n");
      client.subscribe(Control_Topic);
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

// ***** Format RIOT2 Sensor Data message for BIOT2 Base Station *****
void Publish_Sensor_Data()
{
  // ***** Define variables *****
  char TE_b[10], HU_b[10], PR_b[10], AN_b[10], AL_b[10], D1_b[10], SE_b[10];

  // Convert floating point vars into character strings
  dtostrf(Temperature, 5, 1, TE_b);
  dtostrf(Humidity, 5, 1, HU_b);
  dtostrf(Pressure, 5, 1, PR_b);
  dtostrf(Analog, 4, 0, AN_b);
  dtostrf(Alarm, 1, 0, AL_b);
  dtostrf(Digital_1, 1, 0, D1_b);
  dtostrf(Update_Sequence, 6, 0, SE_b);
  
  strcpy(Sensor_Data, "NI,\0");
  strncat(Sensor_Data, Node_Id, 23);
  strcat(Sensor_Data, ",SW,");
  strncat(Sensor_Data, Code_Version, 5);
  strcat(Sensor_Data, ",TE,");
  strncat(Sensor_Data, TE_b, 5);
  strcat(Sensor_Data, ",HU,");
  strncat(Sensor_Data, HU_b, 5);
  strcat(Sensor_Data,  ",PR,");
  strncat(Sensor_Data, PR_b, 5);
  strcat(Sensor_Data, ",SE,");
  strncat(Sensor_Data, SE_b, 6);
  strcat(Sensor_Data, ",AN,");
  strncat(Sensor_Data, AN_b, 4);
  strcat(Sensor_Data, ",AL,");
  strncat(Sensor_Data, AL_b, 1);  
  strcat(Sensor_Data, ",D1,");
  strncat(Sensor_Data, D1_b, 1); 

  
  // Ensure a connection exists with the MQTT server on the Pi
  Serial.print("-> Checking connection to MQTT server...");
  if (!client.connected()) reconnect();
  else Serial.println("connected");
  
  // Publish Sensor Data to MQTT message queue
  Serial.println("-> Sending Sensor Data to MQTT queue");
  Serial.println(Sensor_Data);
  
  client.publish(Sensor_Topic, Sensor_Data);
  
  // Update Record Sequence
  Update_Sequence++;
  if (Update_Sequence > 999999)
    Update_Sequence = 0; 
}

