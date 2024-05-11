/* ************************ RCS TR40 RS485 to MQTT bridge ************************
 * This is to bridge the RCS TR40 thermostat to an MQTT server. The folllowing are
 * the topics that are available for subscription to set the noted settings on the 
 * thermostat.
 * *******************************************************************************
 * 
 * This is the thermostat setpoint used in single setpoint systems
 * casa_de_bemo/living_room/rcs_tr40_thermostat/SP
 * 
 * This is the thermostat heating setpoint used in dual setpoint systems
 * casa_de_bemo/living_room/rcs_tr40_thermostat/SPH
 * 
 * This is the thermostat cooling setpoint used in dual setpoint systems
 * casa_de_bemo/living_room/rcs_tr40_thermostat/SPC
 * 
 * This is the thermostat mode. Format: 
 * M=a, where “a” = O (Off); H (Heat); C (Cool); A (Auto); EH (Emergency Heat).
 * casa_de_bemo/living_room/rcs_tr40_thermostat/M
 * 
 * This is the fan mode. Format: 
 * F=x, where “x“ = 0 (Off or Auto) or 1(On).
 * casa_de_bemo/living_room/rcs_tr40_thermostat/F
 * 
 * This is used to display a text message on the Wall Disply Unit (WDU). Format: 
 * TM=”abcde...” where “a...” = character string, 80 characters max or #.
 * Text must be enclosed in double quotes. String may include spaces and carriage 
 * returns, but may not contain double quotes.
 * casa_de_bemo/living_room/rcs_tr40_thermostat/TM
 * 
 * This is to send an outside temperature to be displayed on the WDU
 * casa_de_bemo/living_room/rcs_tr40_thermostat/OT
 * 
 * This is used to set the time on the WDU. Format: 
 * TIME=hh:mm:ss, where hh (00-23), mm (00-59), ss (00-59).
 * casa_de_bemo/living_room/rcs_tr40_thermostat/TIME
 * 
 * This is used to set the date on the WDU. Format: 
 * DATE=mm/dd/yy, where mm (01-12), dd (1-31), yy (00-99).
 * casa_de_bemo/living_room/rcs_tr40_thermostat/DATE
 * 
 * This is used to set the day of the week on the WDU. Format: 
 * DOW=n, where n = 1-7, Sun=1, Sat=7.
 * casa_de_bemo/living_room/rcs_tr40_thermostat/DOW
 * 
 * *******************************************************************************
 * An R=(1 or 2) command is used to request the status of the thermostat
 * 
 * An R=1 command returns temperature, setpoint, and mode data.
 * Format: A=00 O=1 OA=88 Z=1 T=77 SP= 70 SPH=70 SPC=78 M=H FM=0
 * 
 * OA=xxx - Outside Air where xxx = temperature in degrees.
 * Z =xx - Zone where “x” =zone number. (not used in this bridge)
 * T =xxx - Current Temperature where xxx = -64 to 191 degrees.
 * SP=xx - Current SetPoint where xx is 40 to 113 degrees F.
 * SPH=xx - Heating SetPoint where xx = 40 to 109 degrees F.
 * SPC=xx - Cooling SetPoint where xx = 44 to 113 degrees F.
 * M=a - Mode where a = O (Off), H(Heat), C(Cool), A(Auto), EH(Emergency Heat) 
 *       or I (Invalid).
 * FM=x - Fan Mode where x = 0 (Off or Auto) or 1 (On).
 * 
 * An R=2 command returns the state of the control unit’s HVAC output status
 * This is NOT relay outputs but “calls” i.e., stage 1 heat, stage 2 cool. It also 
 * returns multi-zone system information which is not implemented in this bridge.
 * Format: A=00 O=1 H1A=0 H2A=0 H3A=0 C1A=0 C2A=0 FA=0 VA=0 SM=A SF=0
 * 
 * H1A=x - Heating stage 1 where x = 0 for Off or 1 for Heating Stage 1 On
 * H2A=x - Heating stage 2 where x = 0 for Off or 1 for Heating Stage 2 On
 * H2A=x - Heating stage 3 where x = 0 for Off or 1 for Heating Stage 3 On
 * C1A=x - Cooling stage 1 where x = 0 for Off or 1 for Cooling Stage 1 On
 * C2A=x - Cooling stage 2 where x = 0 for Off or 1 for Cooling Stage 2 On
 * FA=x - Fan status where x = 0 for Off (same as Auto) or 1 for Manual Fan On.
 * SCP=xy - Staging delays, where MOT/MRT is Minimum Off Time/Minimum Run TIme 
 *          x = Stage 1 Call, 0 for Off or 1 for MOT On or 2 for MRT On.
 *          y = Stage 2 Call, 0 for Off or 1 for MOT On or 2 for MRT On.
 * 
 * SM=a - System Mode where a = O (Off), H (Heat), C (Cool).
 */

#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <SoftwareSerial.h>
#include <iostream>
#include <map>
#include <string>
using namespace std;

// Declare Constants and Pin Numbers
#define SSerialRX            21  //Serial Receive pin 8
#define SSerialTX            20  //Serial Transmit pin 7

#define SSerialTxControl     4  //RS485 Direction control

#define RS485Transmit        HIGH
#define RS485Receive         LOW

// put function declarations here:
void setup_wifi();
void callback(char*, byte*, unsigned int);
void reconnect();
void sendCmd(String);
void parseReceived(String);
void parseStatus(String, String);
void print(String);
void println(String);


// Replace the next variables with your SSID/Password combination
const char* ssid = "phpwebscripting3";
const char* password = "password";

// Replace the next variables with your MQTT username/Password combination
const char* MQTTUser = "mqtt-ha-user";
const char* MQTTPass = "MqttPa$$word1!";

// Add your MQTT Broker IP address, example:
//const char* mqttServer = "192.168.1.144";
const char* mqttServer = "192.168.1.117";

//Subscribe topics.  These are the things we are allowing to be set
const char* setTempTopic = "casa_de_bemo/living_room/rcs_tr40_thermostat/SP/set";
const char* setHeatTopic = "casa_de_bemo/living_room/rcs_tr40_thermostat/SPH/set";
const char* setCoolTopic = "casa_de_bemo/living_room/rcs_tr40_thermostat/SPC/set";
const char* setModeTopic = "casa_de_bemo/living_room/rcs_tr40_thermostat/M/set";
const char* setFanModeTopic = "casa_de_bemo/living_room/rcs_tr40_thermostat/FM/set";
const char* setTextMessageTopic = "casa_de_bemo/living_room/rcs_tr40_thermostat/TM/set";
const char* setScheduleControlTopic = "casa_de_bemo/living_room/rcs_tr40_thermostat/SC/set";
const char* setOutsideTempTopic = "casa_de_bemo/living_room/rcs_tr40_thermostat/OT/set";
const char* setTimeTopic = "casa_de_bemo/living_room/rcs_tr40_thermostat/TIME/set";
const char* setDateTopic = "casa_de_bemo/living_room/rcs_tr40_thermostat/DATE/set";
const char* setDayOfWeekTopic = "casa_de_bemo/living_room/rcs_tr40_thermostat/DOW/set";
const char* debugMode = "casa_de_bemo/living_room/rcs_tr40_thermostat/debug";

//Publish topics.  This is the data that the thermostat will present to Home Assistant
const char* connectionTopic = "casa_de_bemo/living_room/rcs_tr40_thermostat/status/LWT";
const char* availabilityTopic = "casa_de_bemo/living_room/rcs_tr40_thermostat/availability";
const char* setpointHeatTopic = "casa_de_bemo/living_room/rcs_tr40_thermostat/SPH";
const char* setpointCoolTopic = "casa_de_bemo/living_room/rcs_tr40_thermostat/SPC";
const char* currentTempTopic = "casa_de_bemo/living_room/rcs_tr40_thermostat/T";
const char* outsideAirTopic = "casa_de_bemo/living_room/rcs_tr40_thermostat/OA";
const char* modeTopic = "casa_de_bemo/living_room/rcs_tr40_thermostat/M";
const char* fanModeTopic = "casa_de_bemo/living_room/rcs_tr40_thermostat/FM";
//These topics are being commented for now to try and use the action topic from the 
//MQTT HVAC integration.
/*
const char* heatStage1Topic = "casa_de_bemo/living_room/rcs_tr40_thermostat/H1A";
const char* heatStage2Topic = "casa_de_bemo/living_room/rcs_tr40_thermostat/H2A";
const char* heatStage3Topic = "casa_de_bemo/living_room/rcs_tr40_thermostat/H3A";
const char* coolStage1Topic = "casa_de_bemo/living_room/rcs_tr40_thermostat/C1A";
const char* coolStage2Topic = "casa_de_bemo/living_room/rcs_tr40_thermostat/C2A";
*/
const char* actionTopic = "casa_de_bemo/living_room/rcs_tr40_thermostat/action";
const char* fanStatusTopic = "casa_de_bemo/living_room/rcs_tr40_thermostat/FA";
const char* stagingDelaysTopic = "casa_de_bemo/living_room/rcs_tr40_thermostat/SCP";
const char* systemModeTopic = "casa_de_bemo/living_room/rcs_tr40_thermostat/SM";
const char* scheduleControlTopic = "casa_de_bemo/living_room/rcs_tr40_thermostat/SC";

// DECLARE VARIABLES 
//the last sent heat set point
String lastSetpointHeat;
//the last sent cool set point
String lastSetpointCool;
//The last mode received from the thermostat (O, H, C, A, EH)
String lastMode = "";
//The last fan mode received from the thermostat
String lastFanMode = "";
//The last temperature received from the thermostat
String lastTemp = "";
//The last action based on the heating or cooling stage
String lastAction = "";
//This is the RS485 address of this node that gets sent to the thermostat
String serialAddr = "1";
//The originator code identifier of the originator of the message
String originator = "00";
//Placeholder for the last command sent
String lastSent = "";
//The command to be sent to the thermostat
String command = "";
//Counter used in place of sleep to request information at certain intervals
int updateCounter = 0;
int refreshCounter = 0;
//The last outside air value received for display on the WDU
float lastOutsideAir = 0;
//Tell whether a command was sent or not.
boolean commandSent = false;
//Tells the code when to send a refresh of the received RS485 data sent from the thermostat
boolean refresh = true;
boolean processedR1 = false;
boolean processedR2 = false;

//Tells whether we are reconnecting to MQTT or not
boolean reconnecting = false;
//Setting this to true will print debug messages to the serial port
boolean debugPrint = false;
//The number of reconnect attempts.  Used to reset if over a certain number of tries
int reconnectCount = 0;

std::map<int, String> st = {{-4, "MQTT CONNECTION TIMEOUT"},
                            {-3, "MQTT CONNECTION LOST"},
                            {-2, "MQTT CONNECT FAILED"},
                            {-1, "MQTT DISCONNECTED"},
                            {0, "MQTT CONNECTED"},
                            {1, "MQTT CONNECT BAD PROTOCOL"},
                            {2, "MQTT CONNECT BAD CLIENT ID"},
                            {3, "MQTT CONNECT UNAVAILABLE"},
                            {4, "MQTT CONNECT BAD CREDENTIALS"},
                            {5, "MQTT CONNECT UNAUTHORIZED"}};

// Declare objects
SoftwareSerial RS485Serial(SSerialRX, SSerialTX); // RX, TX
WiFiClient espClient;
PubSubClient client(espClient);

/**
 * @brief the main setup method for instantiating objects
 * 
 */
void setup() {
  Serial.begin(115200);
  // Start the software serial port, to another device
  RS485Serial.begin(9600);   // set the data rate

  setup_wifi();
  client.setServer(mqttServer, 1883);
  client.setCallback(callback);
}

/**
 * @brief the main loop code
 * 
 */
void loop() {
  // put your main code here, to run repeatedly:

  String dataIn;
  
  if (RS485Serial.available()) {
    dataIn = RS485Serial.readStringUntil('\n');
    parseReceived(dataIn);
  }

  if (!client.connected()) {
    reconnecting = true;
    reconnect();
  }

  // The updateCounter is used to request information at different intervals. Requesting 
  // information at different intervals helps prevent data errors on the serial transmission.
  updateCounter ++;
  refreshCounter ++;
  
  if (updateCounter == 100000) {
    command = "R=1";
  } else if (updateCounter == 200000) {
    command = "R=2";
    updateCounter = 0;
  }
  //every minute we should send a data refresh
  if (refreshCounter == 600000) {
    refresh = true;
    refreshCounter = 0;
  }

  if (command != "") {
    sendCmd(command);
  }

  client.loop();
}

/**
 * @brief Set up the wifi object
 * 
 */
void setup_wifi() {
  delay(10);
  // We start by connecting to a WiFi network
  println("");
  print("Connecting to ");
  println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    print(".");
  }

  println("");
  println("WiFi connected");
  println("IP address: ");
  println((String)WiFi.localIP());
}

/**
 * @brief This is to process incoming messages
 * 
 * @param topic The topic we are subscribed to
 * @param message The payload for that topic
 * @param length The length of the payload string
 */
void callback(char* topic, byte* message, unsigned int length) {
  print("Message arrived on topic: ");
  print(topic);
  print(". Message: ");
  String messageTemp;
  boolean numeric = false;
  boolean doubleQuotes = false;
  
  for (int i = 0; i < length; i++) {
    if (isDigit((char)message[i])) {
      numeric = (i == 0 || numeric) ? true : false;
    } else {
      numeric = false;
    }
    if ((char)message[i] == '"') {
      doubleQuotes = (!doubleQuotes) ? false : true;
    }
    //print(message[i].str());
    messageTemp += (char)message[i];
  }
  println(messageTemp);

  //Check to see if the received message is for one of our thermostat control topics
  
  //Both the SetHeatTopic (SPH) and setCoolTopic (SPC) also check the setTempTopic. When
  //checking the setTempTopic it will check the last mode of the thermostat and perform 
  //the same actions for SPH when the mode is "H" and SPC when the mode is "C".  This
  //allows for a singleplace for setting the temperature based on what mode it is in.

  //SPH=xx - Heating SetPoint where xx = 40 to 109 degrees F.
  if (String(topic) == setHeatTopic || (String(topic) == setTempTopic && lastMode =="H")) {
    println("Set the Heating SetPoint");
    if (numeric && messageTemp.toInt()>39 && messageTemp.toInt() < 110) {
      command = "SPH=" + messageTemp;
      updateCounter = 0;
    } else {
      if(!numeric) {
        println("Must be a number");
      } else {
        println("The heating setpoint must be a number between 40 and 109");
      }
    }
  } else 
  //SPC=xx - Cooling SetPoint where xx = 44 to 113 degrees F.
  if (String(topic) == setCoolTopic || (String(topic) == setTempTopic && lastMode =="C")) {
    println("Set the Cooling SetPoint");
    if (numeric && messageTemp.toInt()>43 && messageTemp.toInt() < 114) {
      command = "SPC=" + messageTemp;
      updateCounter = 0;
    } else {
      if(!numeric) {
        println("Must be a number");
      } else {
        println("The cooling setpoint must be a number between 44 and 113");
      }
    }
  } else 
  //SC=n - Schedule control where “n” is 0 (Hold); 1 (Run); or ?. Run starts
  //       execution of setpoint schedule, Hold stops schedule execution and 
  //       holds current setpoint..
  if (String(topic) == setScheduleControlTopic) {
    println("Set the schedule control");
    if (messageTemp == "0" || messageTemp == "1" || messageTemp == "?") {
      command = "SC=" + messageTemp;
      updateCounter = 0;
    } else {
      println("The schedule control can only be 0 (Hold), 1 (Run) or ?");
    }
  } else 
  //M=a - Mode where a = O (Off), H(Heat), C(Cool), A(Auto), EH(Emergency Heat) or I (Invalid).
  if (String(topic) == setModeTopic) {
    println("Set the Mode");
    if (messageTemp == "O" || messageTemp == "H" || messageTemp == "C" || messageTemp == "A" || messageTemp == "EH" || messageTemp == "I") {
      command = "M=" + messageTemp;
      updateCounter = 0;
    } else {
      println("The mode must be one of O, H, C, A, EH or I");
    }
  } else 
  //FM=x - Fan Mode where x = 0 (Off or Auto) or 1 (On).
  if (String(topic) == setFanModeTopic) {
    println("Set the Fan Mode");
    if (messageTemp == "0" || messageTemp == "1") {
      command = "FM=" + messageTemp;
      updateCounter = 0;
    } else {
      println("The fan mode must be either 0 or 1");
    }
  } else 
  //TM=”abcde...” where “a...” = character string, 80 characters max or #.
  if (String(topic) == setTextMessageTopic) {
    println("Set the Text Message");
    if (length <= 80 && !doubleQuotes) {
      command = "TM=" + messageTemp;
      updateCounter = 0;
    } else {
      if (length > 80) {
        println("The text message must be less than 80 characters");
      } else {
        println("The text message cannot contain double quotes");
      }
    }
  } else 
  //OT=xxx, where “xxx” = temperature in degrees.
  if (String(topic) == setOutsideTempTopic) {
    println("Set the Outside Temperature");
    if (numeric && messageTemp.toInt()>= -50 && messageTemp.toInt() < 125) {
      command = "OT=" + messageTemp;
      updateCounter = 0;
    } else {
      if(!numeric) {
        println("Must be a number");
      } else {
        println("The outside temp must be a number between -50 and 125");
      }
    }
  } else 
  //TIME=hh:mm:ss, where hh is hours (00-23), mm is minutes (00-59), ss is seconds (00-59).
  if (String(topic) == setTimeTopic) {
    println("Set the Time");
    command = "TIME=" + messageTemp;
    updateCounter = 0;
  } else 
  //DATE=mm/dd/yy, where mm is month (01-12), dd is day of month (1-31), yy is year (00-99).
  if (String(topic) == setDateTopic) {
    println("Set the Date");
    command = "DATE=" + messageTemp;
    updateCounter = 0;
  } else 
  //DOW=n, where n = 1-7, Sun=1, Sat=7
  if (String(topic) == setDayOfWeekTopic) {
    println("Set the Day of the week");
    command = "DOW=" + messageTemp;
    updateCounter = 0;
  }
  //
  if (String(topic) == debugMode) {
    debugPrint = (messageTemp == "on") ? true : false;
  }
}

/**
 * @brief Used to reconnect to the MQTT server
 * 
 */
void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("ESP8266Client", MQTTUser, MQTTPass)) {
      println("connected");
      // Subscribe

      client.subscribe(setTempTopic);
      client.subscribe(setHeatTopic);
      client.subscribe(setCoolTopic);
      client.subscribe(setModeTopic);
      client.subscribe(setFanModeTopic);
      client.subscribe(setScheduleControlTopic);
      client.subscribe(setTextMessageTopic);
      client.subscribe(setOutsideTempTopic);
      client.subscribe(setTimeTopic);
      client.subscribe(setDateTopic);
      client.subscribe(setDayOfWeekTopic);
      client.subscribe(debugMode);

      client.publish(connectionTopic, "Connected");
      client.publish(availabilityTopic, "available");
      reconnectCount = 0;
      reconnecting = false;
    } else {
      print("failed, rc=");
      print(st[client.state()]);
      println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
      reconnectCount ++;
      println("reconnect count = " + String (reconnectCount));
      //Check if we had 6 reconnecton attempts (30 seconds) and if so, restart 
      //the controller to refresh things
      if (reconnectCount == 6) {
        ESP.restart();
      }
    }
  }
}

/**
 * @brief Used to parse data received from the RS485 network connected to the thermostat
 * 
 * @param Message The message string to parse
 */
void parseReceived(String Message) {
  
  String StatusData;
  //int Count;
  String StatusString;
  String Status;
  String Type;
  String Value;

  int Index;
  int Start;
  int End;
  int StatIndex;

  Index = Message.indexOf(' ');
  Start = 0;
  
  println("Message: " + Message);
  
  //Check if the message came from the originator (the thermostat) and is for 
  //the serial address of this node
  if (Message.startsWith("A=" + originator + " O=" + serialAddr)) {
    while (End != Message.length()) {
      End = (Index == -1) ? Message.length() : Index;
      //Get the status string to process
      StatusString = Message.substring(Start, End);
      //Change our start position to 1 over the last space found
      Start = Index + 1;
      //Find the end of the next status string
      Index = Message.indexOf(' ', Start);
  
      //Now we need to process the status string into it's Type and Value
      StatIndex = StatusString.indexOf('=');
      Type = StatusString.substring(0, StatIndex);
      Value = StatusString.substring(StatIndex + 1);
      parseStatus(Type, Value);
    } 
    //Clear the commandSent
    commandSent = false;
    //Check if we processed an R1 status message
    if (command == "R=1") {
      processedR1 = true;
    }
    //Check if we processed an R2 status message
    if (command == "R=2") {
      processedR2 = true;
    }
    //When on a refresh cycle we need to make sure we process both R1 and R2 status 
    //messages before we consider the refresh complete and set the refresh to false
    if (processedR1 && processedR2) {
      refresh = processedR1 = processedR2 = false;
    }
    command = "";
    println("");
    println("Response received and processed");
    println("");
  } else {
    sendCmd(lastSent);
  }
  
} //End parseReceived

/**
 * @brief Used to parse a status parameter sent from the thermostat
 * 
 * @param type The status type
 * @param Value The status value
 */
void parseStatus(String Type, String Value) {  
  if (Type == "OA") {
    //Outside Air - only publish if the value has changed
    if (lastOutsideAir != Value.toFloat() || refresh)
      client.publish(outsideAirTopic, Value.c_str());
    lastOutsideAir = Value.toFloat();
  } else if (Type == "T") {
    //Current temperature - only publish if the value has changed
    println("Current temperature=" + Value);
    if (lastTemp != Value || refresh)
      client.publish(currentTempTopic, Value.c_str());
    lastTemp = Value;
  } else if (Type == "SP") {
    //Set Point (for single setpoint systems)  Set the heating or cooling depending 
    //on what mode we are in
    if ((lastMode == "H" || lastMode == "EH") && (lastSetpointHeat != Value || refresh)) {
      println("Single setpoint Heat=" + Value);
      client.publish(setpointHeatTopic, Value.c_str());
      lastSetpointHeat = Value;
    } else if (lastMode == "C" && (lastSetpointCool != Value || refresh)) {
      println("Single setpoint cool=" + Value);
      client.publish(setpointCoolTopic, Value.c_str());
      lastSetpointCool = Value;
    }
  } else if (Type == "SPH") {
    //Heating set point
    println("Heating set point=" + Value);
    if (lastSetpointHeat != Value || refresh)
      client.publish(setpointHeatTopic, Value.c_str());
    lastSetpointHeat = Value;
  } else if (Type == "SPC") {
    //Cooling set point
    println("Cooling set point=" + Value);
    if (lastSetpointCool != Value || refresh)
      client.publish(setpointCoolTopic, Value.c_str());
    lastSetpointCool = Value;
  } else if (Type == "M") {
    //RCS thermostat mode 
    println("mode=" + Value);
    println("Set energy mode to normal");
    if (lastMode != Value || refresh) {
      client.publish(modeTopic, Value.c_str());
      lastMode = Value;
      if (Value == "O") {
          println("Set to Off");
      } else if (Value == "H") {
          println("Set to Heating");
      } else if (Value == "C") {
          println("Set to Cooling");
      } else if (Value == "A") {
          println("Set to AutoChangeOver");
      } else if (Value == "EH") {
          println("Set to Emergency Heat");
      }
    }
  } else if (Type == "FM") {
    //RCS current fan mode (0=off 1=on)
    if (lastFanMode != Value || refresh) {
      client.publish(fanModeTopic, Value.c_str());
      lastFanMode = Value;
      if (Value == "1") {
        println("Set fan mode to ContinuousOn");
      } else {
        println("Set fan mode to Auto");
      }
    }
  //Type 2 status message types
  //{% set values = {'O':'Off', 'H1':'Stage 1 heating', 'H2':'Stage 2 heating', 'H3':'Stage 3 heating', 'C1':'Stage 1 heating', 'C2':'Stage 2 cooling', 'I':'Idle', 'F':'Fan'} %}
  } else if (Type == "H1A" && Value == "1" && (lastAction != "H1" || refresh)) {
    //RCS heating stage 1
    println("Set to heating Stage 1 Min");
    client.publish(actionTopic, "H1");
    lastAction = "H1";
  } else if (Type == "H2A" && Value == "1" && (lastAction != "H2" || refresh)) {
    //RCS heating stage 2
    println("Set to heating Stage 2 Normal");
    client.publish(actionTopic, "H2");
    lastAction = "H2";
  } else if (Type == "H3A" && Value == "1" && (lastAction != "H3" || refresh)) {
    //RCS heating stage 3
    println("Set to heating Stage 3 Max");
    client.publish(actionTopic, "H3");
    lastAction = "H3";
  } else if (Type == "C1A" && Value == "1" && (lastAction != "C1" || refresh)) {
    //RCS cooling stage 1
    println("Set to cooling Stage 1 Normal");
    client.publish(actionTopic, "C1");
    lastAction = "C1";
  } else if (Type == "C2A" && Value == "1" && (lastAction != "C2" || refresh)) {
    //RCS cooling stage 2
   println("Set to cooling Stage 2 Max");
    client.publish(actionTopic, "C2");
    lastAction = "C2";
  // We may receive values of 0 for both C1A and H1A we may get an on/Auto cycling if one or the 
  // other is on. To prevent this we must verify the mode we are in and only set it to auto if 
  // the heating or cooling stage with a 0 value matches the mode we are in
  } else if (((Type == "C1A" && lastMode == "C") || (Type == "H1A" && (lastMode == "H" || lastMode == "EH"))) && Value == "0" && (lastAction != "O" || refresh)) {
    //send(msgFanStatus.set("Off"));
    println("Set fan status to off");
    client.publish(actionTopic, "O");
    lastAction = "O";
  } else if (Type == "FA" && (lastAction != "F" || refresh)) {
    //RCS fan status
     //client.publish(fanStatusTopic, Value.c_str()); 
     client.publish(actionTopic, "F"); 
    lastAction = "F";
    if (Value == "1") {
      println("Set flow mode to ContinuousOn");
    } else {
      println("Set fan speed to auto");
    }
  } else if (Type == "SC") {
    //RCS schedule control
     client.publish(scheduleControlTopic, Value.c_str());
    if (Value == "0") {
      println("Schedule control is set to Hold");
    } else if (Value == "1") {
      println("Schedule control is set to Run");
    }else {
      println("Unknown schedule control response");
    }
  } else if (Type == "VA") {
    //Vent damper not used
  } else if (Type == "D1") {
    //Damper #1 not used
  } else if (Type == "SCP") {
    //This is for MOT (Minimum Off Time) and MRT (Minimum Run Time) statuses for 
    //stages 1 and 2.  I may find a way to implement this, but for now it is not used
    
    //String stg1 = Value.substring(0, 1);
    //String stg2 = Value.substring(0, Value.length() - 1);
  }
  
} //End parseStatus

/**
 * @brief Sends a command out to the RS485 network
 * 
 * @param cmd The command to send
 */
void sendCmd(String cmd) {
  //Assemble the command string using the defined serial address and originator codes
  String commandStr = "A=" + serialAddr + " O=" + originator + " " + cmd;
  println("");
  println("Command sent: " + commandStr);
  // Enable RS485 Transmit only for the duration of the send
  digitalWrite(SSerialTxControl, RS485Transmit);
  RS485Serial.print(commandStr + "\r");
  lastSent = cmd;
  //Return to RS485 receive mode  
  digitalWrite(SSerialTxControl, RS485Receive);
  delay(50);
  commandSent = true;
  command = "";
} //End sendCmd

/**
 * @brief used to easily turn on and off debug printing to the serial port with Serial.print
 * 
 * @param Message The string message to print to the serial port
 */
void print(String Message) {
  if (debugPrint) 
    Serial.print(Message);
}

/**
 * @brief used to easily turn on and off debug printing to the serial port with Serial.println
 * 
 * @param Message The string message to print to the serial port 
 */
void println(String Message) {
  if (debugPrint) 
    Serial.println(Message);
}
