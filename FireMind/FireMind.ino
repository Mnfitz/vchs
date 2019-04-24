
#include <RFM69_ATC.h>
#include <RFM69_OTA.h>
#include <RFM69.h>
#include <RFM69registers.h>

#include <SPIFlash.h>
#include <LowPower.h>

#include <Wire.h>
#include <EEPROM.h>
#include <assert.h>

//This is confusing.

//We want to define our own main(), however the Arduino SDK
//creates its own main() that fights us. 

//So we define these dummy setup() and loop() routines
//here to trick the Arduino SDK into thinking 
//we're using its main().

//Instead, we use our own main() in FireMind_Main.cpp.
//Ours takes precedence because of the file order in:
//.\Build\sketch\FireMind.ino.cpp.d

void setup()
{
	assert(!"We never arrive here because we define our own main() in FireMide_Main.cpp");
}

void loop()
{
	assert(!"We never arrive here because we define our own main() in FireMide_Main.cpp");
}