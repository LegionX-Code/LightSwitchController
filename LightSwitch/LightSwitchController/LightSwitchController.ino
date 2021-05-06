#include <SPI.h>                  // For networking
#include <Ethernet.h>             // For networking
#include <PubSubClient.h>         // For MQTT
#include "Wire.h"                 // For MAC address ROM
#include "DHT.h"                  // For temperature / humidity sensor
#include <FTOLED.h>               // For OLED display
#include <fonts/SystemFont5x7.h>  // For OLED display
#include "config.h"

/**
 * MQTT callback
 */
void callback(char* topic, byte* payload, unsigned int length)
{
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i=0;i<length;i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
}

// Instantiate MQTT client
//PubSubClient client(broker, 1883, callback);
EthernetClient ethclient;
PubSubClient client(ethclient);

void reconnect() {
  
  // Loop until we're reconnected
  while (!client.connected()) {
    
    Serial.print("Attempting MQTT connection...");
    
    // Attempt to connect
    String clientString = "Reconnecting Arduino-" + String(Ethernet.localIP());
    clientString.toCharArray(clientBuffer, clientString.length()+1);
    
    if (client.connect(panelName, mqtt_user, mqtt_pass)) {
      
      Serial.println("connected");
      // Once connected, publish an announcement...
      clientString.toCharArray(clientBuffer, clientString.length() + 1);
      client.publish(statusTopic, clientBuffer);
      // ... and resubscribe
      //client.subscribe("inTopic");
    } else {
      
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

/* ************************************************************************************* */
/* Button setup */
static byte lastButtonState[48] = {   0,  0,  0,  0,    0,  0,  0,  0,
                                      0,  0,  0,  0,    0,  0,  0,  0,
                                      0,  0,  0,  0,    0,  0,  0,  0,
                                      0,  0,  0,  0,    0,  0,  0,  0,
                                      0,  0,  0,  0,    0,  0,  0,  0,
                                      0,  0,  0,  0,    0,  0,  0,  0 };


byte lastButtonPressed         = 0;
#define DEBOUNCE_DELAY 50
/* ************************************************************************************* */

/**
 * Initial configuration
 */
void setup()
{
  Serial.begin(9600);  // Use the serial port to report back readings
    
  /* Set up the watchdog timer */
  if(ENABLE_EXTERNAL_WATCHDOG == true)
  {
    pinMode(WATCHDOG_PIN, OUTPUT);
    digitalWrite(WATCHDOG_PIN, LOW);
  }
  
  /* Set up the tilt sensor */
  pinMode(TILT_SENSOR_PIN, INPUT_PULLUP);
  delay(100);
  
  if( ENABLE_OLED == true )
  {
    oled.begin();
    if(digitalRead(TILT_SENSOR_PIN) == LOW)
    {
      Serial.println("Vertical alignment");
      oled.setOrientation(ROTATE_90);
    } else {
      Serial.println("Horizontal alignment");
    }
    oled.selectFont(SystemFont5x7);
    box.setForegroundColour(DODGERBLUE);

    box.print(OLEDMessage);
  }
  
  Wire.begin();        // Wake up I2C bus
  
  if( ENABLE_MAC_ADDRESS_ROM == true )
  {
    Serial.print(F("Getting MAC address from ROM:"));
    mac[0] = readRegister(0xFA);
    mac[1] = readRegister(0xFB);
    mac[2] = readRegister(0xFC);
    mac[3] = readRegister(0xFD);
    mac[4] = readRegister(0xFE);
    mac[5] = readRegister(0xFF);
  } else {
    Serial.print(F("Using static MAC address: "));
  }
  // Print the IP address
  char tmpBuf[17];
  sprintf(tmpBuf, "%02X:%02X:%02X:%02X:%02X:%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  Serial.println(tmpBuf);
  
  if( ENABLE_OLED == true )
  {
    box.println(tmpBuf);
  }
  
  // Set up the Ethernet library to talk to the Wiznet board
  if( ENABLE_DHCP == true )
  {
    Ethernet.begin(mac);      // Use DHCP
  } else {
    Ethernet.begin(mac, ip);  // Use static address defined above
  }
  
  // Print IP address:
  Serial.print(F("My IP: http://"));
  for (byte thisByte = 0; thisByte < 4; thisByte++) {
    // print the value of each byte of the IP address:
    Serial.print(Ethernet.localIP()[thisByte], DEC);
    if( thisByte < 3 )
    {
      Serial.print(".");
    }
  }
  
  
  Serial.println();
  Serial.println(Ethernet.localIP());
  
  if( ENABLE_OLED == true )
  {
      box.println(F("       "));
      box.println(F("     Item IP:"));
      box.print("  ");
      for (byte thisByte = 0; thisByte < 4; thisByte++) {
      // print the value of each byte of the IP address:
      box.print(Ethernet.localIP()[thisByte], DEC);
      if( thisByte < 3 )
      {
        box.print(".");
      }
    }
    box.println();
  }
  
  Serial.println("Setting input pull-ups");
  for( byte i = 0; i < 48; i++)
  {
    pinMode(buttonArray[i], INPUT_PULLUP);
    Serial.print(buttonArray[i]);
    Serial.print(" ");
  }
  Serial.println();

  /* Connect to MQTT broker */
  Serial.println("connecting...");
  client.setServer(broker, 1883);
  client.setCallback(callback);
  String clientString = "Starting Arduino-" + Ethernet.localIP();
  clientString.toCharArray(clientBuffer, clientString.length() + 1);
  client.publish(statusTopic, clientBuffer);
  
  Serial.println("Ready.");
}


/**
 * Main program loop
 */
void loop()
{
  if (!client.connected()) {
    reconnect();
  }
  box.print(OLEDMessage);
  if( ENABLE_OLED == true )
  {
    if( millis() > (lastActivityTime + (1000 * oled_timeout)))
    {
      oled.setDisplayOn(false);
    }
  }

  runHeartbeat();
  
  client.loop();
  
  byte i;
  for( i = 0; i < 48; i++) {
    processButtonDigital( i );
  }
}


/**
 * The heartbeat takes care of both patting the watchdog and reporting sensor values
 */
void runHeartbeat()
{
  if((millis() - watchdogLastResetTime) > WATCHDOG_RESET_INTERVAL)  // Is it time to run yet?
  {
    humidity = dht.readHumidity();
    temperature = dht.readTemperature();

    char tempC[10];
    dtostrf(temperature,1,2,tempC);
    char relH[10];
    dtostrf(humidity,1,2,relH);

    client.publish(temperatureTopic, tempC);
    if(client.publish(humidityTopic, relH))
    {
      patWatchdog();  // Only pat the watchdog if we successfully published to MQTT
    }
    Serial.print("T: ");
    Serial.print(temperature, DEC);
    Serial.print("C H:");
    Serial.print(humidity, DEC);
    Serial.println("%");
    // The interval timer is updated inside patWatchdog()
  }
}


/**
 */
void processButtonDigital( byte buttonId )
{
    int sensorReading = digitalRead( buttonArray[buttonId] );
    
    if( sensorReading == 0 )  // Input pulled low to GND. Button pressed.
    {
      //Serial.println( "Button pressed" );
      if( lastButtonState[buttonId] == 0 )   // The button was previously un-pressed
      {
        if((millis() - lastActivityTime) > DEBOUNCE_DELAY)  // Proceed if we haven't seen a recent event on this button
        {
          lastActivityTime = millis();
          if( ENABLE_OLED == true )
          {
            oled.setDisplayOn(true);
          }
    
          lastButtonPressed = buttonId;
          Serial.print( "transition on ");
          Serial.print( buttonId, DEC );
          Serial.print(" (input ");
          Serial.print( buttonArray[buttonId + 1] );
          Serial.println(")");
        
          String messageString = String(panelId) + "-" + String(buttonArray[buttonId]);
          messageString.toCharArray(messageBuffer, messageString.length()+1);
        
          //String topicString = "device/" + String(panelId) + "/button";
          String topicString = "buttons";
          topicString.toCharArray(topicBuffer, topicString.length()+1);
  
          //client.publish(topicBuffer, messageBuffer);
        
          client.publish("buttons", messageBuffer);
          if( ENABLE_OLED == true )
          {
            box.setForegroundColour(LIMEGREEN);
            box.print(F("Button pressed: "));
            box.println(buttonId + 1);
          }
        }
      } else {
        // Transition off
        //digitalWrite(statusArray[buttonId-1], LOW);
        //digitalWrite(13, LOW);
      }
      lastButtonState[buttonId] = 1;
    }
    else {
      lastButtonState[buttonId] = 0;
    }
}



/**
 * Pulse the hardware watchdog timer pin to reset it
 */
void patWatchdog()
{
  if( ENABLE_EXTERNAL_WATCHDOG )
  {
    digitalWrite(WATCHDOG_PIN, HIGH);
    delay(WATCHDOG_PULSE_LENGTH);
    digitalWrite(WATCHDOG_PIN, LOW);
  }
  watchdogLastResetTime = millis();
}


/**
 * Helper function used to read the MAC address ROM via I2C
 */
byte readRegister(byte r)
{
  unsigned char v;
  Wire.beginTransmission(MAC_I2C_ADDRESS);
  Wire.write(r);  // Register to read
  Wire.endTransmission();

  Wire.requestFrom(MAC_I2C_ADDRESS, 1); // Read a byte
  while(!Wire.available())
  {
    // Wait
  }
  v = Wire.read();
  return v;
}
