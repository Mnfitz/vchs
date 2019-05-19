
#include <Arduino.h>
#include <RFM69.h>
#include <RFM69_ATC.h>
#include <SPIFlash.h>

#define ENCRYPTKEY    "sampleEncryptKey" //exactly the same 16 characters/bytes on all nodes!
#define FREQUENCY     RF69_433MHZ
#define NODEID        1    //unique for each node on same network
#define NETWORKID     100  //the same on all nodes that talk to each other
#define SERIAL_BAUD   115200

#ifdef __AVR_ATmega1284P__
  #define LED           15 // Moteino MEGAs have LEDs on D15
  #define FLASH_SS      23 // and FLASH SS on D23
#else
  #define LED           9 // Moteinos have LEDs on D9
  #define FLASH_SS      8 // and FLASH SS on D8
#endif

SPIFlash flash(FLASH_SS, 0xEF30); //EF30 for 4mbit  Windbond chip (W25X40CL)

class Recv
{
public:
    void Setup();
    void Loop();
    bool GetCoords(GpsCoords& outGpsCoords);
    
private:
    void Blink(byte PIN, int DELAY_MS);
    float BuffToCoord(volatile uint8_t inBuff[12]);

private:
    byte      mAckCount{0};
    uint32_t  mPacketCount{0};
    bool      mPromiscuousMode{false}; //set to 'true' to sniff all packets on the same network
    GpsCoords mGpsCoords{};
    bool      mIsCoordReady{false};

#ifdef ENABLE_ATC
    RFM69_ATC mRadio{};
#else
    RFM69     mRadio{};
#endif
};

inline void Recv::Setup() 
{
  Serial.begin(SERIAL_BAUD);
  delay(10);
  mRadio.initialize(FREQUENCY,NODEID,NETWORKID);
#ifdef IS_RFM69HW_HCW
  mRadio.setHighPower(); //must include this only for RFM69HW/HCW!
#endif
  mRadio.encrypt(ENCRYPTKEY);
  mRadio.promiscuous(mPromiscuousMode);
  //mRadio.setFrequency(919000000); //set frequency to some custom frequency
  char buff[50];
  sprintf(buff, "\nListening at %d Mhz...", FREQUENCY==RF69_433MHZ ? 433 : FREQUENCY==RF69_868MHZ ? 868 : 915);
  Serial.println(buff);
  if (flash.initialize())
  {
    Serial.print("SPI Flash Init OK. Unique MAC = [");
    flash.readUniqueId();
    for (byte i=0;i<8;i++)
    {
      Serial.print(flash.UNIQUEID[i], HEX);
      if (i!=8) Serial.print(':');
    }
    Serial.println(']');
    
    //alternative way to read it:
    //byte* MAC = flash.readUniqueId();
    //for (byte i=0;i<8;i++)
    //{
    //  Serial.print(MAC[i], HEX);
    //  Serial.print(' ');
    //}
  }
  else
    Serial.println("SPI Flash MEM not found (is chip soldered?)...");
    
#ifdef ENABLE_ATC
  Serial.println("RFM69_ATC Enabled (Auto Transmission Control)");
#endif
}

inline void Recv::Loop()
{
  //Serial.println("Recv Loop");
  //process any serial input
#if 0
  if (Serial.available() > 0)
  {
    char input = Serial.read();
    if (input == 'r') //d=dump all register values
      mRadio.readAllRegs();
    if (input == 'E') //E=enable encryption
      mRadio.encrypt(ENCRYPTKEY);
    if (input == 'e') //e=disable encryption
      mRadio.encrypt(null);
    if (input == 'p')
    {
      mPromiscuousMode = !mPromiscuousMode;
      mRadio.promiscuous(mPromiscuousMode);
      Serial.print("Promiscuous mode ");Serial.println(mPromiscuousMode ? "on" : "off");
    }
  }
#endif 

  if (mRadio.receiveDone())
  {
    Serial.print("#[");
    Serial.print(++mPacketCount);
    Serial.print(']');
    Serial.print('[');Serial.print(mRadio.SENDERID, DEC);Serial.print("] ");
    if (mPromiscuousMode)
    {
      Serial.print("to [");Serial.print(mRadio.TARGETID, DEC);Serial.print("] ");
    }
    for (byte i = 0; i < mRadio.DATALEN; i++)
    //for (byte i = 0; i < 50; i++)
      Serial.print((char)mRadio.DATA[i]);
      
      mGpsCoords.mLatitude = BuffToCoord(&mRadio.DATA[0]);
      mGpsCoords.mLongitude = BuffToCoord(&mRadio.DATA[12]);
      mGpsCoords.mAltitude = BuffToCoord(&mRadio.DATA[24]);

      Serial.println();
      Serial.print("GPS Coords: ");
      Serial.print(mGpsCoords.mLatitude);
      Serial.print(", ");
      Serial.print(mGpsCoords.mLongitude);
      Serial.print(", ");
      Serial.println(mGpsCoords.mAltitude);
      
      const bool isValidCoord = (mGpsCoords.mLatitude != 0
                                && mGpsCoords.mLongitude != 0
                                && mGpsCoords.mAltitude != 0);
 
      if (isValidCoord)
      {
        mIsCoordReady = true;
      }
      

    //Serial.print("   [RX_RSSI:");Serial.print(mRadio.RSSI);Serial.print("]");
    
    if (mRadio.ACKRequested())
    {
      byte theNodeID = mRadio.SENDERID;
      mRadio.sendACK();
      Serial.print(" - ACK sent.");

      // When a node requests an ACK, respond to the ACK
      // and also send a packet requesting an ACK (every 3rd one only)
      // This way both TX/RX NODE functions are tested on 1 end at the GATEWAY
      if (mAckCount++%3==0)
      {
        Serial.print(" Pinging node ");
        Serial.print(theNodeID);
        Serial.print(" - ACK...");
        delay(3); //need this when sending right after reception .. ?
        if (mRadio.sendWithRetry(theNodeID, "ACK TEST", 8, 0))  // 0 = only 1 attempt, no retries
          Serial.print("ok!");
        else Serial.print("nothing");
      }
    }
    Serial.println();
    Blink(LED,3);
  }
}

inline void Recv::Blink(byte PIN, int DELAY_MS)
{
  pinMode(PIN, OUTPUT);
  digitalWrite(PIN,HIGH);
  delay(DELAY_MS);
  digitalWrite(PIN,LOW);
}

inline bool Recv::GetCoords(GpsCoords& outGpsCoords)
{
  if (mIsCoordReady)
  {
    outGpsCoords = mGpsCoords;
    mIsCoordReady = false;
    return (true);
  }

  return (false);
}

inline float Recv::BuffToCoord(volatile uint8_t inBuff[12])
{
#if 1
  //Watch out! The character buffer holds a 4 byte float type
  float result = *reinterpret_cast<volatile float*>(&inBuff[0]);
  return result;
#else
  int index=0;
  int tempCoord=0;
  char tempChar=0;
  float sign = 1.0;

  while (index<12)
  {
    tempChar=inBuff[index];
    
    if (isdigit(tempChar))
    {
      tempChar -= '0'; //This line converts ASCII code digits into base 10 digits
      tempCoord *= 10;
      tempCoord += tempChar;
      
    }
    else if (tempChar == '-')
    {
      sign = -1.00;
    }
    
    index++;
  } 
  tempCoord /= 100000000; //Assumes target data uses 12-1-3=8 digits of precision
  return tempCoord * sign;
#endif
}
