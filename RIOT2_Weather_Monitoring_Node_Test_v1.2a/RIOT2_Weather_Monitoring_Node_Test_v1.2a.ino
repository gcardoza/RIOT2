//  Project:  RIOT2 - Remote IOT Node for monitoring Weather - Temperature, Humidity, Barometric Pressure
//  Author:   Geofrey Cardoza
//  Baseline: August 31st, 2016
//  Revision: November 4th, 2016 - Removed DHT22 and BMP180, and added Temperature offset for BME280
//
//  Hardware Configuration:
//    AdaFruit Feather Huzzah with ESP8266 Micro-controller
//      - WiFi & MQTT messaging interface
//    BME280 for Temperature, Humidity and Pressure (Primary)
//    Analog (0-1V) Moisture Sensor
//    Alarm - digital Input Sensor
//    Digital 1 input sensor

  const char* Code_Version = " 1.2a";

// ***** Include header files *****
  #include "RioT_Test.h"            // RioT & BioT Security Data
  #include <PubSubClient.h>         // Library for MQTT Pub/Sub functions
  #include <ESP8266WiFi.h>          // Library for ESP8266 WiFi microcontroller
  #include <Wire.h>                 // Library for I2C Communication
  #include <Adafruit_Sensor.h>
  #include <Adafruit_BME280.h>
  
// ***** Declare global variables for the RIOT2 Node
  const char* Node_Type = "RIOT2";
  char Node_Id[30];
  int Update_Interval = 60;            // set default Update interval to 60 sec. (can be changed)
  float Temperature, Humidity, Pressure, Altitude, Analog, Alarm, Digital_1;
  unsigned long Update_Sequence = 0;      // Update sequence number to base
  char Sensor_Data[150];                  // Buffer to hold formatted sensor data payload
  
// ***** Define variables to track how long the program has been running *****
  unsigned long Current_Time;
  unsigned long Last_Publish_Time = 0;

// ***** MQTT pub/sub service info *****
  const char* RioT_Status_Topic = "/RioT/Status";
  const char* Control_Topic = "/Control/RioT";
  WiFiClient espClient;
  PubSubClient client(espClient);
  char Control_Command[10];        // Inbound subscribed control message command
  int  Control_Data;               // Inbound subscribed control message data
  
// ***** Set BME280 Temperature, Humidity and Pressure Variables *****
  #define SEALEVELPRESSURE_HPA (1013.25)
  int Home_Altitude = 317;        // Altitude in meters to compensate for Barometric Pressure 
  Adafruit_BME280 bme;            // I2C
  float Temp_Offset = -1.5;       // The temperature offset (if BME reads 1.5 show 0)
 
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
    Serial.println("  -> BME280: Could not find sensor.");
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
  Last_Publish_Time = millis()/1000 - Update_Interval;
}


// ********** MAIN PROGRAM LOOP **********
void loop()
{
  // ***** Wait Update Interval prior to Publishing Sensor Data *****
  Current_Time = millis()/1000;    // get current program run=time in ms
  
  if (Current_Time >= Last_Publish_Time + Update_Interval)
  {
    Serial.println("\n***** PREPARING TO READ AND PUBLISH SENSOR DATA *****");
    Last_Publish_Time = Current_Time;   // Reste last publish time

    // ***** Read Sensors and Store Data in Global variables *****
    read_BME280();          // Temerature, Humidity & Pressure
    read_Analog_Digital();  // Analog, Alarm and Digital 1

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
  float T, P, Correction_Factor;        // Var to adjust altitude correction factor for pressure
  Serial.println("-> BME280: Reading Temperature, Humidity, and Pressure");

  T = bme.readTemperature();
  Temperature = T + Temp_Offset;
  P = bme.readPressure()/(1000.0);                  // Convert to kPa
  Humidity = bme.readHumidity();
  Altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);
 
  // Correct Pressure based on current altitude read bythe BME280
  Correction_Factor = (760-(Home_Altitude*3.281*0.026))/760;  // Corrects Pressure to Sea Level
  Pressure = P/Correction_Factor;
  
  Serial.print("  -> Actual Temp = ");
  Serial.print(T);
  Serial.print(" *C, Hum = ");
  Serial.print(Humidity);
  Serial.print(" %, Pres = ");
  Serial.print(Pressure);
  Serial.print("\n  -> Corrected Temp = ");
  Serial.print(Temperature);
  Serial.print(" *C, Alt = ");
  Serial.print(Altitude);
  Serial.print(" m, Correction Factor = ");
  Serial.println(Correction_Factor);
  return(0);
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

// ***** Reconnect to MQTT service and subscribe to Control Topic *****
void reconnect()
{
  // Loop until we're reconnected
  while (!client.connected())
  {
    Serial.print("\n  -> Attempting MQTT connection with ID: ");
    Serial.print(MQTT_Id);
    Serial.print(", PW:************");
    
    // Attempt to connect to the MQTT server
    if (client.connect(Node_Id, MQTT_Id, MQTT_Pw)) 
    {
      Serial.println(" ... connected");
      
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

// ***** Process Subscribed message received from BioT *****
void callback(char* topic, byte* payload, unsigned int length)
{  
  char data_b[100], Control_Message[150];
  int i, x, y;
  float n;
  
  // Flash LED for 0.5 seconds to indicate message arrival - GPIO 0
  digitalWrite(BUILTIN_LED, LOW);   // Note LOW turns LED on
  delay(500);
  digitalWrite(BUILTIN_LED, HIGH);  // Turn the LED off  
  
  Serial.println("\n***** Subscribed Topic Message Arrived *****");
  Serial.print ("-> Topic: ");
  Serial.print(topic);
  Serial.print(", Message Length: ");
  Serial.println(length);
  
  // Ensure the message is long enough to contain a valid command
  if(length <3) 
  {
    Serial.println("-> Control Message too short. Ignoring Command");
    return;
  }
  
  // Convert Payload to a character array
  for (int i = 0; i < length; i++) Control_Message[i] = (char)payload[i];
  Control_Message[length] = '\0'; // Ensure there is a string terminator at the end
  Serial.print("-> Message: <");
  Serial.print(Control_Message);
  Serial.println(">");

  // ***** Process Inbound Command *****
  // Isolate the Control Command (first 3 characters)
  Serial.print("-> Processing Inbound Control Command: ");
  strncpy(Control_Command, Control_Message, 3);
  Control_Command[3] = '\0';
  Serial.println(Control_Command);

  // ** Change Status Update Interval **
  if(!strcmp(Control_Command, "UF:"))
  {
    //Ensure Control_Message length is correct for this message
    if(length < 7) 
    {
      Serial.println("   -> UF: Message is too short. Ignoring command.");
      return;
    }
    strncpy(data_b, &Control_Message[3] ,4); 
    data_b[4] = '\0';
    Control_Data = atoi(data_b);
    
    Serial.print("   -> Changing Update Interval to ");
    Serial.print(Control_Data);
    Serial.println(" seconds");
    if(Control_Data < 5)
    {
      Serial.print ("      -> Value < 5 seconds. Forcing Interval to 5 seconds\n");
      Control_Data = 5;
    }
    Update_Interval = Control_Data;    // Set Status Update Interval
    Last_Publish_Time = millis()/1000 - Update_Interval; // Force sending a status
    return;
  }

  // ** Force Node to Publish a Status NOW **
  if(!strcmp(Control_Command, "UN:"))
  {
    Serial.println("   -> Forcing Node to Publish Status Now");
    Last_Publish_Time = millis()/1000 - Update_Interval;
    return;
  }
  
  // ** Force Node to Reset the Update Sequence number to 0 **
  if(!strcmp(Control_Command, "RS:"))
  {
    Serial.println("   -> Forcing Node to Reset Sequence Number to 0");
    Update_Sequence = 0;
    Serial.println("   -> Forcing Node to Publish Status Now");
    Last_Publish_Time = millis()/1000 - Update_Interval;  // Force sending a status
    return;
  }
  
  // ** Change Temperature Offset to compensate for BME280 self heating **
  if(!strcmp(Control_Command, "TO:"))
  {
    //Ensure Control_Message length is correct for this message
    if(length < 8) 
    {
      Serial.println("   -> TO: Message is too short. Ignoring command.");
      return;
    }
    strncpy(data_b, &Control_Message[3] ,5); 
    data_b[5] = '\0';
    n = atof(data_b);

    Serial.print("   -> Changing Temperature Offset to ");
    Serial.print(n);
    Serial.println(" *C");
    
    Temp_Offset = n;    // Set Temperature Offset
    Last_Publish_Time = millis()/1000 - Update_Interval; // Force sending a status
    return;
  }
  // Command was not found.
  Serial.println("   -> Unknown Control Command. Ignoring it.");
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
  
  strcpy(Sensor_Data, "NI:\0");
  strncat(Sensor_Data, Node_Id, 23);
  strcat(Sensor_Data, ",SW:");
  strncat(Sensor_Data, Code_Version, 5);
  strcat(Sensor_Data, ",SE:");
  strncat(Sensor_Data, SE_b, 6);
  strcat(Sensor_Data, ",SD:");
  strcat(Sensor_Data, ",TE:");
  strncat(Sensor_Data, TE_b, 5);
  strcat(Sensor_Data, ",HU:");
  strncat(Sensor_Data, HU_b, 5);
  strcat(Sensor_Data,  ",PR:");
  strncat(Sensor_Data, PR_b, 5);
  strcat(Sensor_Data, ",AN:");
  strncat(Sensor_Data, AN_b, 4);
  strcat(Sensor_Data, ",AL:");
  strncat(Sensor_Data, AL_b, 1);  
  strcat(Sensor_Data, ",D1:");
  strncat(Sensor_Data, D1_b, 1); 

  
  // Ensure a connection exists with the MQTT server on the Pi
  Serial.print("-> Checking connection to MQTT server...");
  if (!client.connected()) reconnect();
  else Serial.println("connected");
  
  // Publish Sensor Data to MQTT message queue
  Serial.println("-> Sending Sensor Data to MQTT queue");
  Serial.println(Sensor_Data);
  
  client.publish(RioT_Status_Topic, Sensor_Data);
  
  // Update Record Sequence
  Update_Sequence++;
  if (Update_Sequence > 999999)
    Update_Sequence = 0; 
}

