
#include <Arduino.h>
#include <RFM69.h>
#include <RFM69_ATC.h>
#include <SPIFlash.h>
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>

#define ENCRYPTKEY    "sampleEncryptKey" //exactly the same 16 characters/bytes on all nodes!
#define FREQUENCY     RF69_433MHZ
#define NODEID        1    //unique for each node on same network
#define NETWORKID     100  //the same on all nodes that talk to each other
#define GATEWAYID   1

#ifdef __AVR_ATmega1284P__
  #define LED           15 // Moteino MEGAs have LEDs on D15
  #define FLASH_SS      23 // and FLASH SS on D23
#else
  #define LED           9 // Moteinos have LEDs on D9
  #define FLASH_SS      8 // and FLASH SS on D8
#endif

//This is tricky.

//The Moteino MEGA can't talk to the Adafruit GPS using SoftwareSerial
//because SoftwareSerial is too flakey. Instead we talk to Adafruit GPS
//using HardwareSerial: Serial1. It appears that you must use Serial1
//because Serial0 is already used(?) by Arduino IDE's upload and terminal
//window. Beware: This means all serial ports on Moteino MEGA have now been
//used up.

HardwareSerial mySerial = Serial1; //Motieno MEGA requires this!
Adafruit_GPS GPS(&Serial1); //Same here!
#define GPSECHO  false

#ifdef ENABLE_ATC
  RFM69_ATC radio;
#else
  RFM69 radio;
#endif

typedef struct __attribute__((packed))
{
	//int           nodeId; //store this nodeId
	//unsigned long uptime; //uptime in ms
	//float         temp;   //temperature maybe?
	char mTargetLat[7];
	char mTargetLon[7];
	char mTargetAlt[7];

} Payload;
Payload theData;
static_assert(sizeof(Payload)==3*7, "Oops, payload isn't 21 bytes in size.");

long lastPeriod = -1;
int TRANSMITPERIOD = 300; //transmit a packet to gateway so often (in ms)

class Target
{
public:
	void Setup();
	void Loop();
	void useInterrupt(boolean v); // Func prototype keeps Arduino 0023 happy

	boolean usingInterrupt = false;

private:
	void Blink(byte PIN, int DELAY_MS);
};



inline void Target::Setup()  
{
	radio.initialize(FREQUENCY,NODEID,NETWORKID);
#ifdef IS_RFM69HW_HCW
	radio.setHighPower(); //must include this only for RFM69HW/HCW!
#endif
	radio.encrypt(ENCRYPTKEY);
	//radio.setFrequency(919000000); //set frequency to some custom frequency

	//Auto Transmission Control - dials down transmit power to save battery (-100 is the noise floor, -90 is still pretty good)
	//For indoor nodes that are pretty static and at pretty stable temperatures (like a MotionMote) -90dBm is quite safe
	//For more variable nodes that can expect to move or experience larger temp drifts a lower margin like -70 to -80 would probably be better
	//Always test your ATC mote in the edge cases in your own environment to ensure ATC will perform as you expect
#ifdef ENABLE_ATC
	radio.enableAutoPower(ATC_RSSI);
#endif


	// connect at 115200 so we can read the GPS fast enough and echo without dropping chars
	// also spit it out
	Serial.begin(115200);
	Serial.println("Adafruit GPS library basic test!");

	// 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
	GPS.begin(9600);

	// uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
	GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
	// uncomment this line to turn on only the "minimum recommended" data
	//GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
	// For parsing data, we don't suggest using anything but either RMC only or RMC+GGA since
	// the parser doesn't care about other sentences at this time

	// Set the update rate
	GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate
	// For the parsing code to work nicely and have time to sort thru the data, and
	// print it out we don't suggest using anything higher than 1 Hz

	// Request updates on antenna status, comment out to keep quiet
	GPS.sendCommand(PGCMD_ANTENNA);

	// the nice thing about this code is you can have a timer0 interrupt go off
	// every 1 millisecond, and read data from the GPS for you. that makes the
	// loop code a heck of a lot easier!
	useInterrupt(true);

	delay(1000);
	// Ask for firmware version
	mySerial.println(PMTK_Q_RELEASE);
}


// Interrupt is called once a millisecond, looks for any new GPS data, and stores it
SIGNAL(TIMER0_COMPA_vect) {
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
#ifdef UDR0
  if (GPSECHO)
    if (c) UDR0 = c;  
    // writing direct to UDR0 is much much faster than Serial.print 
    // but only one character can be written at a time. 
#endif
}

inline void Target::useInterrupt(boolean v) 
{
  if (v) {
    // Timer0 is already used for millis() - we'll just interrupt somewhere
    // in the middle and call the "Compare A" function above
    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);
    usingInterrupt = true;
  } else {
    // do not call the interrupt function COMPA anymore
    TIMSK0 &= ~_BV(OCIE0A);
    usingInterrupt = false;
  }
}

uint32_t timer = millis();

inline void Target::Loop()                     // run over and over again
{
	if (radio.receiveDone())
 	{
		Serial.print('[');Serial.print(radio.SENDERID, DEC);Serial.print("] ");
		for (byte i = 0; i < radio.DATALEN; i++)
		{
			Serial.print((char)radio.DATA[i]);
		}
		Serial.print("   [RX_RSSI:");Serial.print(radio.RSSI);Serial.print("]");
		if (radio.ACKRequested())
		{
			radio.sendACK();
			Serial.print(" - ACK sent");
		}
		Blink(LED,3);
		Serial.println();
  	}

	int currPeriod = millis()/TRANSMITPERIOD;
	if (currPeriod != lastPeriod)
	{
		//fill in the struct with new values
		//theData.nodeId = NODEID;
		//theData.uptime = millis();
		//theData.temp = 91.23; //it's hot!

		Serial.print("Sending struct (");
		Serial.print(sizeof(theData));
		Serial.print(" bytes) ... ");

		if (radio.sendWithRetry(GATEWAYID, (const void*)(&theData), sizeof(theData)))
		{
			Serial.print(" ok!");
		}
		else 
		{
			Serial.print(" nothing...");
		}
		Serial.println();
		Blink(LED,3);
		lastPeriod=currPeriod;
	}

  // in case you are not using the interrupt above, you'll
  // need to 'hand query' the GPS, not suggested :(
  if (! usingInterrupt) {
    // read data from the GPS in the 'main loop'
    char c = GPS.read();
    // if you want to debug, this is a good time to do it!
    if (GPSECHO)
      if (c) Serial.print(c);
  }
  
  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences! 
    // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
    //Serial.println(GPS.lastNMEA());   // this also sets the newNMEAreceived() flag to false
  
    if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
      return;  // we can fail to parse a sentence in which case we should just wait for another
  }

  // if millis() or timer wraps around, we'll just reset it
  if (timer > millis())  timer = millis();

  // approximately every 2 seconds or so, print out the current stats
  if (millis() - timer > 2000) { 
    timer = millis(); // reset the timer
    
    Serial.print("\nTime: ");
    Serial.print(GPS.hour, DEC); Serial.print(':');
    Serial.print(GPS.minute, DEC); Serial.print(':');
    Serial.print(GPS.seconds, DEC); Serial.print('.');
    Serial.println(GPS.milliseconds);
    Serial.print("Date: ");
    Serial.print(GPS.day, DEC); Serial.print('/');
    Serial.print(GPS.month, DEC); Serial.print("/20");
    Serial.println(GPS.year, DEC);
    Serial.print("Fix: "); Serial.print((int)GPS.fix);
    Serial.print(" quality: "); Serial.println((int)GPS.fixquality); 
    if (GPS.fix) {
      Serial.print("Location: ");
      Serial.print(GPS.latitude, 4); Serial.print(GPS.lat);
      Serial.print(", "); 
      Serial.print(GPS.longitude, 4); Serial.println(GPS.lon);
      Serial.print("Location (in degrees, works with Google Maps): ");
      Serial.print(GPS.latitudeDegrees, 4);
      Serial.print(", "); 
      Serial.println(GPS.longitudeDegrees, 4);
      
      Serial.print("Speed (knots): "); Serial.println(GPS.speed);
      Serial.print("Angle: "); Serial.println(GPS.angle);
      Serial.print("Altitude: "); Serial.println(GPS.altitude);
      Serial.print("Satellites: "); Serial.println((int)GPS.satellites);
	  Blink(LED, 3);
    }
  }
}

inline void Target::Blink(byte PIN, int DELAY_MS)
{
  pinMode(PIN, OUTPUT);
  digitalWrite(PIN,HIGH);
  delay(DELAY_MS);
  digitalWrite(PIN,LOW);
}

Target gTarget{};
void setup()
{
  gTarget.Setup();  
}

void loop()
{
  gTarget.Loop();  
}
