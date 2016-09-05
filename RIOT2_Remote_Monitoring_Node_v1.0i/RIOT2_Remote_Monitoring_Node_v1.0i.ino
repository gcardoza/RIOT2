//  Project:  RIOT2 - Remote IOT Node for monitoring Weather - Temperature, Humidity, Barometric Pressure
//  Author:   Geofrey Cardoza
//  Baseline: August 31st, 2016
//  Revision: September 3rd, 2016
//
//  Hardware Configuration:
//    AdaFruit Feather Huzzah with ESP8266 Microcontroller
//      - WiFi & MQTT messaging interface
//    DHT22 for Temperature and Pressure
//    BMP180 for Pressure
//    Analog (0-1V) Moisture Sensor
//    Digital input 1 and 2

  const char* Code_Version = " 1.0i";

// ***** Include header files *****
  #include <PubSubClient.h>         // Library for MQTT Pub/Sub functions
  #include <ESP8266WiFi.h>          // Library for ESP8266 WiFi microcontroller
  #include "DHT.h"                // Library for DHT22 Temperature and Humidity sensor
  #include <Wire.h>                 // Library for I2C Communication
  #include <SFE_BMP180.h>           // Library for BMP180 Pressure and Temperature sensor
  #include <Adafruit_Sensor.h>
  #include <Adafruit_BME280.h>
  
// ***** Declare global variables for the RIOT2 Node
  int   Update_Interval = 10000;           // set default Data Update frequence 20 seconds (BIOT can change it)
  float Temperature, Humidity, Pressure, Analog_1;
  float  Digital_1, Digital_2;
  unsigned long Update_Sequence = 0;      // Update sequence number to base
  char Sensor_Data[150];                  // Buffer to hold formatted sensor data payload

// ***** Define variables to track how long the program has been running *****
  long Current_Time;
  long Last_Publish_Time = 0;

// ***** WiFi & Server Info *****
  #define MQTT_Server "192.168.0.23"
  const char* ssid = "Excal-AS-RC";
  const char* WiFi_Password = "6677889900";
  const char* Node_Type = "RIOT2";
  String Node_Id = "RIOT2-";

// ***** MQTT pub/sub service info *****
  const char* Sensor_Topic = "/RIOT2/SensorData";
  const char* Config_Topic = "/RIOT2/Config";
  WiFiClient espClient;
  PubSubClient client(espClient);

// ***** Set BME280 Temperature, Humidity and Pressure Variables *****
  #define SEALEVELPRESSURE_HPA (1013.25)
  Adafruit_BME280 bme; // I2C
  boolean BME280_Present = true;  // set the default to the board being present
  
// ***** Set DHT22 Temp and Pressure Variables *****
  #define DHT_Type DHT22   // DHT 22  (AM2302), AM2321
  #define DHT22_Pin 2     // The digital IO pin the DHT22 is connected to
  DHT dht(DHT22_Pin, DHT_Type);
  
// ***** Set BMP180 Pressure Sensor Variables *****
  #define ALTITUDE 260.0 // Altitude at Home - 4 Peace Court Caledon ON, Canada
  SFE_BMP180 myBMP180;

// ***** Set Analog and Digital Input Variables *****
  #define Analog1_Pin  0    // Analog input 1 pin
  #define Digital1_Pin 12   // Digital input 1 pin
  #define Digital2_Pin 13  // Digital input 2 pin


// ********** INITIALIZE ALL COMPONENTS OF THE SYSTEM **********
void setup(void)
{
  // Start the serial port for debugging vi Arduino Serial Monitor
  Serial.begin(115200);
  delay (1000); //Pause to allow serial port to initialize
  Serial.print("\n***** STARTING RIOT2 - Code Version: ");
  Serial.print(Code_Version);
  Serial.println(" *****");
  Serial.println("\n***** ENTERING SETUP MODE *****");

  // ***** Start WiFi Communication Subsystem *****
  Serial.print("-> Connecting to WiFi Network\n");
  setup_wifi();

  // ***** Configure & Start MQTT Messaging service *****
  Serial.print("-> MQTT: Configuring Messaging Service\n");
  client.setServer(MQTT_Server, 1883); // Connect to MQTT Server
  client.setCallback(callback);        // Set the callback function when subscribed message arrives 
  client.subscribe(Config_Topic);       // Subscribe to Config Topic
        
  // ***** Configure Onboard LED and set to Off *****
  Serial.print("-> LED: Configuring Onboard LED and set to off\n");
  pinMode(BUILTIN_LED, OUTPUT);
  digitalWrite(BUILTIN_LED, HIGH);  // Turn the LED off by making the voltage HIGH
  
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

  // ***** Configure Onboard LED and set to Off *****
  Serial.print("-> Analog & Digital Input: Configuring A/D and GPIO ports\n");
  pinMode(Digital1_Pin, INPUT_PULLUP);
  pinMode(Digital2_Pin, INPUT_PULLUP);
  
  //Wait a bit before starting the main loop
  delay(2000);
}


// ********** MAIN PROGRAM LOOP **********
void loop()
{
  // Wait Update Interval prior to Publishing Sensor Data
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
}

// *************** Sub-Routines *********************

//  ***** Read the Temperature, Humidity and Pressure from the BME280 over I2C *****
int read_BME280()
{
  Serial.print("-> BME280: Reading Temperature, Humidity, and Pressure");

  Temperature = bme.readTemperature();
  Serial.print("  -> Temperature = ");
  Serial.print(Temperature);
  Serial.print(" *C, ");

  Humidity = bme.readHumidity();
  Serial.print("Humidity = ");
  Serial.print(Humidity);
  Serial.print(" %, ");
  
  Pressure = bme.readPressure() / 10.0F;  // Convert to kPa
  Serial.print("Pressure = ");
  Serial.print(Pressure);
  Serial.println(" kPa");

  return(0);
}

//  ***** Read the Temperature and Humidity from the DHT22 sensor over a serial digital I/O port *****
int read_DHT22()
{
  Serial.print("-> DHT22: Reading Temperature and Humidity");
  
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
  Serial.print(" %, ");
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
          Pressure = myBMP180.sealevel(P,ALTITUDE)/10;  //Convert Pressure to kPa

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

  Analog_1 = analogRead(Analog1_Pin);   // A?D 1 reading in 10 bit resolution. 1V = 1023
 
  if(digitalRead(Digital1_Pin) == HIGH) Digital_1 = 1;
  else Digital_1 = 0;
  
  if(digitalRead(Digital2_Pin) == HIGH) Digital_2 = 1;
  else Digital_2 = 0;
  
  Serial.print("  -> Analog1: ");
  Serial.print(Analog_1);
  Serial.print(" Digital1: ");
  Serial.print(Digital_1);
  Serial.print(" Digital2: ");
  Serial.println(Digital_2);
}

// ***** Connect to the WiFi Network and establish Node Name *****
void setup_wifi()
{
  delay(10);
  // We start by connecting to a WiFi network
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
  uint8_t mac[6];
  WiFi.macAddress(mac);
  Node_Id += macToStr(mac);

  Serial.print("  -> IP address: ");
  Serial.println(WiFi.localIP());
  Serial.print("  -> Node ID: ");
  Serial.println(Node_Id);
}

// ***** Process Subscribed message received from BIOT2 *****
void callback(char* topic, byte* payload, unsigned int length)
{  char message(150);
  
  Serial.println("***** Subscribed Topic Message Arrived *****");
  Serial.print ("  -> Topic: ");
  Serial.print(topic);
  Serial.print(", Message Length: ");
  Serial.println(length);
  Serial.print("  -> Message: ");
  for (int i = 0; i < length; i++) Serial.print((char)payload[i]);
  Serial.println();

  // Flash LED for 0.5 seconds to indicate message arrival
  digitalWrite(BUILTIN_LED, LOW);   // Note LOW turns LED on
  delay(500);
  digitalWrite(BUILTIN_LED, HIGH);  // Turn the LED off

  // *?* Process Subscribed Config Message here

}

// ***** Reconnect to MQTT service and subscribe to Config Topic *****
void reconnect()
{
  // Loop until we're reconnected
  while (!client.connected())
  {
    Serial.print("  -> Attempting MQTT connection...");
    
    // Attempt to connect to the MQTT server
    if (client.connect(Node_Id.c_str())) 
    {
      Serial.println("connected");
      
      // Resubscribe to the MQTT Configuration topic
      Serial.print("  -> Subscribing to MQTT Config.Topic service\n");
      client.subscribe(Config_Topic);
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

// ***** Generate a string from the ESP8266's MAC address *****
String macToStr(const uint8_t* mac)
{
  String result;

  for (int i = 0; i < 6; ++i) 
  {
    result += String(mac[i], 16);
    if (i < 5){
      result += ':';
    }
  }
  return(result);
}

// ***** Format RIOT2 Sensor Data message for BIOT2 Base Station *****
void Publish_Sensor_Data()
{
  // ***** Define variables *****
  char TE_b[10], HU_b[10], PR_b[10], A1_b[10], D1_b[10], D2_b[10], SE_b[10];

  // Convert floating point vars into character strings
  dtostrf(Temperature, 5, 1, TE_b);
  dtostrf(Humidity, 5, 1, HU_b);
  dtostrf(Pressure, 5, 1, PR_b);
  dtostrf(Analog_1, 4, 0, A1_b);
  dtostrf(Digital_1, 1, 0, D1_b);
  dtostrf(Digital_2, 1, 0, D2_b);
  dtostrf(Update_Sequence, 6, 0, SE_b);
  
  strcpy(Sensor_Data, "NI,\0");
  strncat(Sensor_Data, Node_Id.c_str(), 23);
  strcat(Sensor_Data, ",SW,");
  strncat(Sensor_Data, Code_Version, 5);
  strcat(Sensor_Data, ",TE,");
  strncat(Sensor_Data, TE_b, 5);
  strcat(Sensor_Data, ",HU,");
  strncat(Sensor_Data, HU_b, 5);
  strcat(Sensor_Data,  ",PR,");
  strncat(Sensor_Data, PR_b, 5);
  strcat(Sensor_Data, ",A1,");
  strncat(Sensor_Data, A1_b, 4);
  strcat(Sensor_Data, ",D1,");
  strncat(Sensor_Data, D1_b, 1);  
  strcat(Sensor_Data, ",D2,");
  strncat(Sensor_Data, D2_b, 1); 
  strcat(Sensor_Data, ",SE,");
  strncat(Sensor_Data, SE_b, 6);
  
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

