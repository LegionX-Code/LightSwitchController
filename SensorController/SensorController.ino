#include <SPI.h>                  // For networking
#include <Ethernet.h>             // For networking
#include <PubSubClient.h>         // For MQTT
#include <Wire.h>
#include "config.h"
 
void setup() {
  Serial.begin(9600);
  Serial.println("Beginning program");

  Wire.begin();

  // Wait for network connection
  while (Ethernet.begin(device_mac) == 0)
  {
    delay(1000);
  }

  // Configure MQTT
  mqtt.setServer(mqtt_broker, 1883);

  initialisePins();
  initialiseSensors();
}
 
void loop(){
  
  // Keep network connection
  Ethernet.maintain();

  // Reconnect to MQTT server
  if (!mqtt.connected())
  {
    if (!mqttReconnect())
    {
      // Exit early of loop
      return;
    }
  }
  
  checkMovement();
}

void checkMovement(){
  for(int i = 0; i < NUMBER_OF_PINS; i++) {
    
    int val = digitalRead( sensorArray[i] );
    byte pirState = pirStateArray[i];
    
    if (val == HIGH && pirState == LOW) {
        Serial.println("Motion detected!");
  
        sendMessageToMQTT(sensorArray[i]);
        
        pirStateArray[i] = HIGH;
    }
    else if(val == LOW && pirState == HIGH) {
        Serial.println("Motion ended!");
        pirStateArray[i] = LOW;
    }
    
  }
}

void initialiseSensors(){
  for(int i = 0; i < NUMBER_OF_PINS; i++) {
    int val = digitalRead(i);
    pirStateArray[i] = HIGH;
  }
}

void sendMessageToMQTT(int sensorID){
  char str[21];
  itoa(sensorID, str, 10);

  char dest[24];
  strcpy(dest, panelId);
  strcat(dest, "-");
  strcat(dest, str);
  

  String messageString = dest;
  
  Serial.println(messageString);
  
  messageString.toCharArray(messageBuffer, messageString.length()+1);
        
  mqtt.publish("sensors", messageBuffer);
}


/**
 * MQTT reconnect
 * @return bool True on successful connection
 * @return bool False on failure to connect
 */
bool mqttReconnect ()
{
  // Only attempt every 1 seconds to prevent congesting the network
  if (!getTimer(mqtt_timmer, 1000))
  {
    return false;
  }
  
  Serial.println("Attempting to connect to MQTT Broker...");
  
  // Attempt to connect with random client name
  if (mqtt.connect(mqtt_client, mqtt_user, mqtt_pass))
  {
    // Subscribe to topic
    mqtt.publish(statusTopic, clientBuffer);

    Serial.println("Success - Connected to MQTT Broker");
    // Connection was successful
    return true;
  }

  // Still unable to connect
  return false;
}

/**
 * Determine if given timer has reached given interval
 *
 * @param unsigned long tmr The current timer
 * @param int interval Length of time to run timer
 * @return bool True when timer is complete
 * @return bool False when timer is counting
 */
bool getTimer (unsigned long &tmr, int interval)
{
  // Set initial value
  if (tmr < 1)
  {
    tmr = millis();
  }

  // Determine difference of our timer against millis()
  if (millis() - tmr >= interval)
  {
    // Complete. Reset timer
    tmr = 0;
    return true;
  }

  // Still needs to "count"
  return false;
}

/**
 * Initialise Pins
 *
 * @return void
 */
void initialisePins()
{
  Serial.println("Setting input pull-ups");
  for( byte i = 0; i < 48; i++)
  {
    pinMode(sensorArray[i], INPUT_PULLUP);
    Serial.print(sensorArray[i]);
    Serial.print(" ");
  }
}
