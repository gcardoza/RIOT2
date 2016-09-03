//  Project:  RIOT2 - Remote IOT Node for monitoring Weather - Temperature, Humidity, Barometric Pressure
//  Author:   Geofrey Cardoza
//  Baseline: August 31st, 2016  v1.0
//  Revision: August 31st, 2016  v1.0
//
//  Hardware Configuration:
//    AdaFruit Feather Huzzah with ESP8266 Microcontroller
//      - WiFi & MQTT messaging interface
//    DHT22 for Temperature and Pressure
//    BMP180 for Pressure and Temperature
//    Analog (0-1V) Moisture Sensor


// ***** Include header files *****
  #include <PubSubClient.h>         // Library for MQTT Pub/Sub functions
  #include <ESP8266WiFi.h>          // Library for ESP8266 WiFi microcontroller// *?*  #include <stdio.h>
  #include "DHT.h"                // Library for DHT22 Temperature and Humidity sensor
  #include <Wire.h>                 // Library for I2C Communication
  #include <SFE_BMP180.h>           // Library for BMP180 Pressure and Temperature sensor
  
// ***** Declare global variables for the RIOT2 Node
  int Update_Interval = 5000;           // set Data Update frequence default to 20 seconds (BIOT2 can change it)
  unsigned long Update_Sequence = 0;      // Update sequence number to base

// WiFi & Server Info
  #define MQTT_Server "192.168.0.23"
  const char* ssid = "Excal-AS-RC";
  const char* WiFi_Password = "6677889900";
  const char* Node_Type = "RIOT2";
  const char* Code_Version = " 1.0a";
  String Node_Id = "RIOT2-";

// MQTT service info
  char* Sensor_Topic = "/RIOT2/SensorData";
  char* Config_Topic = "/RIOT2/Config";

  WiFiClient espClient;
  PubSubClient client(espClient);

// ***** Set DHT22 Temp and Pressure Variables *****
  #define DHT_Type DHT22   // DHT 22  (AM2302), AM2321
  #define DHT22_Pin 2     // The digital IO pin the DHT22 is connected to
  DHT dht(DHT22_Pin, DHT_Type);
  float DHT22_Temperature, DHT22_Humidity, DHT22_HeatIndex;
  
// ***** Set BMP180 Pressure Sensor Variables *****
  SFE_BMP180 myBMP180;
  #define ALTITUDE 260.0 // Altitude at Home - 4 Peace Court Caledon ON, Canada
  float BMP180_Pressure, BMP180_Temperature;

// ***** Set Moisture Sensor Variables
  #define Moisture_Pin  1 // Analog input pin
  int Moisture, Moisture_p;


// ********** INITIALIZE ALL COMPONENTS OF THE SYSTEM **********
void setup(void)
{
  // Start the serial port for debugging vi Arduino Serial Monitor
  Serial.begin(115200);
  delay (1000); //Pause to allow serial port to initialize
  Serial.print("\n***** STARTING RIOT2 - Code Version: ");
  Serial.print(Code_Version);
  Serial.print(" *****\n");
  Serial.print("\n***** ENTERING SETUP MODE *****\n");

  // ***** Start WiFi Communication Subsystem *****
  Serial.print("-> Connecting to WiFi Network\n");
  setup_wifi();

  // ***** Configure & Start MQTT Messaging service *****
  Serial.print("-> MQTT: Configuring Messaging Service\n");
  client.setServer(MQTT_Server, 1883); // Connect to MQTT Server
  client.setCallback(callback);        // Set the callback function when subscribed message arrives 
      
  // ***** Configure Onboard LED and set to Off *****
  Serial.print("-> LED: Configuring Onboard LED and set to off\n");
  pinMode(BUILTIN_LED, OUTPUT);
  digitalWrite(BUILTIN_LED, HIGH);  // Turn the LED off by making the voltage HIGH
  
  // ***** Initialize DHT22 Temperature and Humidity sensor *****
  Serial.print("-> DHT22: Starting Temperature & Humidity Sensor\n");
  dht.begin();

  // ***** Initialize BMP180 Pressure sensor *****
  Serial.print(F("-> BMP180: Starting Pressure and Temperature Sensor "));
  myBMP180.begin();
  Serial.print(F("\n"));

  //Wait a bit before starting the main loop
  delay(2000);
}


// ********** MAIN PROGRAM LOOP **********
void loop()
{
  // ***** Define variables *****
  String Sensor_Data;   // Data buffer for message to BIOT Base
  char buffer[150], SW_b[10], ID_b[2], DT_b[10], DH_b[10], DI_b[10], BT_b[10], BP_b[10], M1_b[5], SE_b[10]; //*?*
  
  Serial.print("\n***** ENTERING MAIN PROGRAM LOOP *****\n");
  
  // Ensure a connection exists with the MQTT server on the Pi
  Serial.print("-> Checking connection to MQTT server...");
  if (!client.connected()) reconnect();
  else  Serial.println("connected");
  
  // ***** Get Temperature & Humidity from DHT22 sensor and store in Global Vars *****
  Serial.print("-> DHT22: Reading Temperature and Humidity");
  read_DHT22();
  
  // ***** Get Pressure and Temperature from BMP180 module and store in Global Vars *****
  Serial.println("-> BMP180: Reading Temperature and Pressure");
  read_BMP180();
    
  // ***** 4. Get Moisture Readings from sensor and store in Global Vars *****
  Serial.println("-> Moisture: Reading Sensor");
  Moisture = analogRead(Moisture_Pin);   // Moisture reading in 10 bit resolution. 5V = 1023
  Serial.print("  ->Moisture Reading: ");
  Serial.println(Moisture);
  
  // *?* Moisture_p = convertToPercent(Moisture);  // Moisture reading in percentage. 0 = dry, 100 = soaked

  
  // Format RIOT2 Sensor Data message for BIOT2 Base Station

  dtostrf(DHT22_Temperature, 5, 1, DT_b);
  dtostrf(DHT22_Humidity, 5, 1, DH_b);
  dtostrf(DHT22_HeatIndex, 5, 1, DI_b);
  dtostrf(BMP180_Temperature, 5, 1, BT_b);
  dtostrf(BMP180_Pressure, 5, 1, BP_b);
  dtostrf(Moisture_p, 3, 0, M1_b);
  dtostrf(Update_Sequence, 6, 0, SE_b);
         
  Sensor_Data = Node_Id;
  Sensor_Data += ", SW:";
  Sensor_Data += Code_Version;
  Sensor_Data += ", DT:";
  Sensor_Data += DT_b;
  Sensor_Data += ", DH:";
  Sensor_Data += DH_b;
  Sensor_Data += ", DI:";
  Sensor_Data += DI_b;
  Sensor_Data += ", BT:";
  Sensor_Data += BT_b;
  Sensor_Data += ", BP:";
  Sensor_Data += BP_b;
  Sensor_Data += ", M1:";
  Sensor_Data += M1_b;
  Sensor_Data += ", SE:";
  Sensor_Data += SE_b;

  // Publish Sensor Data to MQTT message queue
  strcpy(buffer, Sensor_Data.c_str());
  Serial.println("-> Sending Sensor Data to MQTT queue");
  Serial.println(Sensor_Data);
  
  client.publish("/RIOT2/SensorData", buffer);
  
  // Update Record Sequence
  Update_Sequence++;
  if (Update_Sequence > 999999)
    Update_Sequence = 0;
  
  // Wait update interval prior to sending next update
  delay(Update_Interval);
}

// *************** Sub-Routines *********************

//  ***** Read the Temperature and Humidity from the DHT22 sensor over a serial digital I/O port *****
int read_DHT22()
{
  // Note Reading temperature or humidity takes between .25 - 2 seconds
  DHT22_Humidity = dht.readHumidity();      // Relative Humidity in %
  DHT22_Temperature = dht.readTemperature();  // Temperature in Celcius
                                            // For Fahrenheit use dht.readTemperature(true)

  // Check if DHT22 reads failed and exit
  if (isnan(DHT22_Humidity) || isnan(DHT22_Temperature))
  {
    Serial.println("  -> Failed to read from DHT22 sensor!");
    //Set the failed read values for the readings
    DHT22_Humidity = 0;
    DHT22_Temperature = 0;
    DHT22_HeatIndex = 0;
    return(-1);
  }

  // Calculate heat index in Celsius (isFahreheit = false)
  DHT22_HeatIndex = dht.computeHeatIndex(DHT22_Temperature, DHT22_Humidity, false);

  Serial.println("  -> DHT22 Read successful");
  Serial.print("  -> Temperature: ");
  Serial.print(DHT22_Temperature);
  Serial.print(" *C, ");
  Serial.print("Humidity: ");
  Serial.print(DHT22_Humidity);
  Serial.print(" %, ");
  Serial.print("Heat index: ");
  Serial.print(DHT22_HeatIndex);
  Serial.println(" *C ");
  return(0);
}

//  ***** Read the Temperature and Pressure from the BMP180 sensor over I2C *****
int read_BMP180()
{
  char status;
  double T,P,p0,a;

  status = myBMP180.startTemperature();
  if (status != 0)
  {
    delay(status);
    status = myBMP180.getTemperature(T);
    if (status != 0)
    { 
      BMP180_Temperature = T;
      status = myBMP180.startPressure(3);
      if (status != 0)
      {
        delay(status);
        status = myBMP180.getPressure(P,T);
        if (status != 0)
        {
          BMP180_Pressure = myBMP180.sealevel(P,ALTITUDE)/10;  //Convert Pressure to kPa

          Serial.print("  -> Temperature: ");
          Serial.print(BMP180_Temperature);
          Serial.print(" *C, Barometric Pressure: ");
          Serial.print(BMP180_Pressure);
          Serial.println(" kPa");
          return(0);
        }
        else
        {
          Serial.print(F("  -> BMP180: ERROR Starting Pressure Measurement"));
          return(-4);
        }
      }
      else
      {
        Serial.print(F("  -> BMP180: ERROR Starting Pressure Measurement"));
        return(-3);
      }
    }
    else
    { 
      Serial.print(F("  -> BMP180: ERROR Reading Temperature"));
      return(-2);
    }
  }
  else
  {  
    Serial.println(F("  -> BMP180: ERROR Starting Temperature Measurement"));
    return(-1);
  }
}

// ***** Convert moisture reading to a percent 0% to 100% *****
int convertToPercent(int analogValue)
{
  int percentValue = 0;
  percentValue = map(analogValue, 164, 65, 0, 100);
  if (percentValue > 100) percentValue = 100;
  if (percentValue < 0) percentValue = 0;
  return percentValue;
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
  Serial.println("");
  
  // Create Node ID = Node_Type-MAC Address
  uint8_t mac[6];
  WiFi.macAddress(mac);
  Node_Id += macToStr(mac);

  Serial.println("  -> WiFi connected");
  Serial.print("  -> IP address: ");
  Serial.println(WiFi.localIP());
  Serial.print("  -> Node ID: ");
  Serial.println(Node_Id);
}

// ***** Process Subscribed message received from BIOT2 *****
void callback(char* topic, byte* payload, unsigned int length)
{
  char message(150);
  
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println(); // *?*

  // Switch on the LED if an 1 was received as first character
  if ((char)payload[0] == '1') {
    digitalWrite(BUILTIN_LED, LOW);   // Turn the LED on (Note that LOW is the voltage level
    // but actually the LED is on; this is because
    // it is acive low on the ESP-01)
  } else {
    digitalWrite(BUILTIN_LED, HIGH);  // Turn the LED off by making the voltage HIGH
  }
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
      client.subscribe("/RIOT2/Config");
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
  return result;
}

