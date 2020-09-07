/*
 * Remote Weather Station Transmitter -- built as an addressed, reliable message
 * transmitter client using the reliable datagram manager class from the Radio
 * Head libreary.  Reads environmental data from a pair of attached sensors 
 * and broadcasts their readings.  Puts the system to sleep between readings to save
 * power.
 * 
 * Author: David Bryant (david@disquisitioner.com)
 *
 * Development Note: For boards with "native" USB support (e.g. not using an FTDI 
 * chip or similar serial bridge), Serial connection may be lost on sleep/wake,
 * and you might not see messages sent via Serial. This setch uses the onboard LED 
 * as an alternate indicator, with different flashing patterns indicating
 * various states of operation (sleep/wake, transmission success/failure).  If you
 * want to maintain Serial communications look for the comment below to replace
 * processor sleep mode with a simple delay().
 * 
 */

#include <SPI.h>
#include <RH_RF69.h>
#include <RHReliableDatagram.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Adafruit_SleepyDog.h>
#include "SparkFun_Si7021_Breakout_Library.h"

//Create Instance of HTU21D or SI7021 temp and humidity sensor
Weather sensor;

// Create the OneWire bus and the DS18B20 sensor on it
const int ds18b20Pin =  5;   // Pin the DS18B20 data lead (DQ) is connected to 
OneWire oneWire(ds18b20Pin);
DallasTemperature ds18b20(&oneWire);

// Take advantage of onboard support for tracking (and reporting) battery voltage
#define VBATPIN A7

// Counters and thresholds
const int napinterval = 8;  // Number of sleep cycles, at about 15sec
                             // each, betwteen transmissions
uint16_t packetnum = 0;      // packet counter, we increment per xmission

/************ Radio Setup ***************/
// Where to send packets to!
#define DEST_ADDRESS   1
// change addresses for each client board, any number :)
#define MY_ADDRESS     2

// Change to 434.0 or other frequency, must match RX's freq!
#define RF69_FREQ 915.0

#if defined (__AVR_ATmega32U4__) // Feather 32u4 w/Radio
  #define RFM69_CS      8
  #define RFM69_INT     7
  #define RFM69_RST     4
  #define LED           13
#endif

#if defined(ADAFRUIT_FEATHER_M0) // Feather M0 w/Radio
  #define RFM69_CS      8
  #define RFM69_INT     3
  #define RFM69_RST     4
  #define LED           13
#endif

#if defined (__AVR_ATmega328P__)  // Feather 328P w/wing
  #define RFM69_INT     3  // 
  #define RFM69_CS      4  //
  #define RFM69_RST     2  // "A"
  #define LED           13
#endif

#if defined(ESP8266)    // ESP8266 feather w/wing
  #define RFM69_CS      2    // "E"
  #define RFM69_IRQ     15   // "B"
  #define RFM69_RST     16   // "D"
  #define LED           0
#endif

#if defined(ESP32)    // ESP32 feather w/wing
  #define RFM69_RST     13   // same as LED
  #define RFM69_CS      33   // "B"
  #define RFM69_INT     27   // "A"
  #define LED           13
#endif

/* Teensy 3.x w/wing
#define RFM69_RST     9   // "A"
#define RFM69_CS      10   // "B"
#define RFM69_IRQ     4    // "C"
#define RFM69_IRQN    digitalPinToInterrupt(RFM69_IRQ )
*/
 
/* WICED Feather w/wing 
#define RFM69_RST     PA4     // "A"
#define RFM69_CS      PB4     // "B"
#define RFM69_IRQ     PA15    // "C"
#define RFM69_IRQN    RFM69_IRQ
*/

// Singleton instance of the radio driver
RH_RF69 rf69(RFM69_CS, RFM69_INT);

// Class to manage message delivery and receipt, using the driver declared above
RHReliableDatagram rf69_manager(rf69, MY_ADDRESS);

void setup() 
{
  Serial.begin(115200);
  //while (!Serial) { delay(1); } // wait until serial console is open, remove if not tethered to computer

  pinMode(LED, OUTPUT);     
  pinMode(RFM69_RST, OUTPUT);
  digitalWrite(RFM69_RST, LOW);

  Serial.println("Feather Addressed RFM69 TX Test!");
  Serial.println();

  // manual reset
  digitalWrite(RFM69_RST, HIGH);
  delay(10);
  digitalWrite(RFM69_RST, LOW);
  delay(10);
  
  if (!rf69_manager.init()) {
    Serial.println("RFM69 radio init failed");
    while (1);
  }
  Serial.println("RFM69 radio init OK!");
  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM (for low power module)
  // No encryption
  if (!rf69.setFrequency(RF69_FREQ)) {
    Serial.println("setFrequency failed");
  }

  // If you are using a high power RF69 eg RFM69HW, you *must* set a Tx power with the
  // ishighpowermodule flag set like this:
  rf69.setTxPower(20, true);  // range from 14-20 for power, 2nd arg must be true for 69HCW

  // The encryption key has to be the same as the one in the server
  uint8_t key[] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
                    0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
  rf69.setEncryptionKey(key);
  
  pinMode(LED, OUTPUT);

  Serial.print("RFM69 radio @");  Serial.print((int)RF69_FREQ);  Serial.println(" MHz");

  // Initialize the I2C sensors and ping them
  sensor.begin();

  // Initialize DS18B20 on the OneWire bus
  ds18b20.begin();
}


// Dont put this on the stack:
uint8_t buf[RH_RF69_MAX_MESSAGE_LEN];
uint8_t data[] = "  OK";

void loop() {
  int sleepMS = 16000;
  unsigned int naps = 0;       // nap counter, we increment each sleep cycle
  float battery, dstempF;
  float itempF = 0; /* Current reading from temperature sensor */
  float humidity = 0;   /* Current reading from humidity sensor */

  // Check the battery
  battery = readbattery();
 
  // Get the temperature and humidity from the Si7021
  humidity   = sensor.getRH();
  // Temperature is measured every time RH is requested.
  // It is faster, therefore, to read it from previous RH
  // measurement with getTemp() instead of with readTemp()
  itempF = sensor.getTempF();

  // Get the temperature from the DS18B20
  ds18b20.requestTemperatures();
  dstempF = 32 + ( (9.0 * ds18b20.getTempCByIndex(0))/5.0 ) ;

  // Now we have all our sensor data, build the payload we want to send
  // Build payload of important info and send it as a single line of text via the radio
  String payload = String(sleepMS) + "," + String(packetnum) + "," + String(battery) + "," +
     String(dstempF) + "," + String(itempF) + "," + String(humidity);
  packetnum++;
  
  // Extract the raw characters from our String payload
  const char *radiopacket = payload.c_str();

  // Serial.print("Sending "); Serial.println(radiopacket);
  
  // Send a message to the DESTINATION!
  if (rf69_manager.sendtoWait((uint8_t *)radiopacket, strlen(radiopacket), DEST_ADDRESS)) {
    // Now wait for a reply from the server
    uint8_t len = sizeof(buf);
    uint8_t from;   
    if (rf69_manager.recvfromAckTimeout(buf, &len, 2000, &from)) {
      buf[len] = 0; // zero out remaining string
      
      // Serial.print("Got reply from #"); Serial.print(from);
      // Serial.print(" [RSSI :");  Serial.print(rf69.lastRssi());  Serial.print("] : ");
      // Serial.println((char*)buf);     
      Blink(LED, 40, 3); //blink LED 3 times, 40ms between blinks
    } else {
      // Serial.println("No reply, is anyone listening?");
      Blink(LED,100,4); // blink LED 4 times, 100ms between blinks
    }
  } else {
    // Serial.println("Sending failed (no ack)");
    Blink(LED,100,4);  // blink LED 4 times, 100ms between blinks
  }
  
  // Enter low power sleep mode -- the watchdog will allow low power 
  // mode for as long as possible.  The actual amount of time spent
  // in sleep will be returned (in milliseconds).  This does, though, disrupt
  // USB operations so confuses any attached host (e.g., as during development).
  // so should instead simulate low power sleep mode with a call to delay().
  rf69.sleep();  // Put radio in low power mode. 
  for(naps=0;naps<napinterval;naps++) {
    // delay(sleepMS);  // If simulating sleep()
    sleepMS = Watchdog.sleep();
  }
  // Code resumes here on after sleeping.
}

float readbattery() {
  float measuredvbat;

  // Check and report battery voltage
  measuredvbat = analogRead(VBATPIN);
  measuredvbat *= 2;    // we divided by 2, so multiply back
  measuredvbat *= 3.3;  // Multiply by 3.3V, our reference voltage
  measuredvbat /= 1024; // convert to voltage
  Serial.print("VBat: " ); Serial.println(measuredvbat);

  return(measuredvbat);
}

void Blink(byte PIN, byte DELAY_MS, byte loops) {
  for (byte i=0; i<loops; i++)  {
    digitalWrite(PIN,HIGH);
    delay(DELAY_MS);
    digitalWrite(PIN,LOW);
    delay(DELAY_MS);
  }
}
