//Her loader vi libraries. De bruges til at hjælpe vores enheder med at forstå kommandoer.
//Disse libraries behandler vores bluetooth enhed
#include <Arduino.h>
#include <SPI.h>
#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"
#include "BluefruitConfig.h"

#if SOFTWARE_SERIAL_AVAILABLE
#include <SoftwareSerial.h>
#endif

#define FACTORYRESET_ENABLE         0
#define MINIMUM_FIRMWARE_VERSION    "0.6.6"
#define MODE_LED_BEHAVIOUR          "MODE"
Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);
//****************************************************************************************************************************************

void error(const __FlashStringHelper*err) {
  Serial.println(err);
  while (1);
}

///****************************************************************************************************************************************
//Her definierer vi vores pinout. VI skal senere konverterer vores input til en integer således at vi kan sende værdien.

//The Serial Clock… an input from your Arduino.
#define SCK_PIN 2
//Chip Select.   Setting low, selects the Module and tells it to supply an output that is synchronize with a clock
#define CS_PIN 3
//The module’s serial output.   Your Arduino will read this output.
#define SO_PIN 5


//****************************************************************************************************************************************
void setup() {
  //Vi starter vore setup, den kører en gang, den definere serial monitorens baud rate og pinmode.
  Serial.begin(9600);
  pinMode(CS_PIN, OUTPUT);
  pinMode(SO_PIN, INPUT);
  pinMode(SCK_PIN, OUTPUT);

  while (!Serial);  // required for Flora & Micro
  delay(500);
  Serial.begin(115200);
  Serial.println(F("Adafruit Bluefruit Command <-> Data Mode Example"));
  Serial.println(F("------------------------------------------------"));

      /* Initialise the module */
  Serial.print(F("Starter bluetooth enheden: "));
   if ( !ble.begin(VERBOSE_MODE) )
  {
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }
  Serial.println( F("OK!") );

  if ( FACTORYRESET_ENABLE )
  {
    /* Perform a factory reset to make sure everything is in a known state */
    Serial.println(F("Performing a factory reset: "));
    if ( ! ble.factoryReset() ) {
      error(F("Couldn't factory reset"));
    }
  }
 ble.echo(false);

  Serial.println("Requesting Bluefruit info:");
  /* Print Bluefruit information */
  ble.info();

  ble.verbose(false);  // debug info is a little annoying after this point!

  Serial.println(F("******************************"));

  // LED Activity command is only supported from 0.6.6
  if ( ble.isVersionAtLeast(MINIMUM_FIRMWARE_VERSION) )
  {
    // Change Mode LED Activity
    Serial.println(F("Change LED activity to " MODE_LED_BEHAVIOUR));
    ble.sendCommandCheckOK("AT+HWModeLED=" MODE_LED_BEHAVIOUR);
  }
  //Give module a new name
  ble.println("AT+GAPDEVNAME=Ultimatewarlord"); // named Ultimatewarlord.

  // Check response status
  ble.waitForOK();

  /* Wait for connection */
  while (! ble.isConnected()) {
    delay(500);
  }
  // Set module to DATA mode
  Serial.println( F("Switching to DATA mode!") );
  ble.setMode(BLUEFRUIT_MODE_DATA);

  Serial.println(F("******************************"));
}

void loop()
{
  //Vi starter vores loop, altså den funktion som gentager sig selv over og over.
  //Vi aflæser den nuværende temperatur.
  float Temp = ReadTemp();
  //Der kan opstå en løs forbindelse, så vi sikre at vores temperaturmåler er forbundet
  //Vi bruger en if sætning til at se om værdien er et tal vi kan forstå eller NA 
  if(Temp != -1)
  //Hvis værdien kan bruges, printer vi værdien som celcius.
    Serial.println(ReadTemp()); 
  //Hvis ikke værdien kan bruges, skriver vi Thermocouple disconnected!
  else
    Serial.println("Thermocouple disconnected!");
  //Vi sætter vores loop til at have et delay på et sekundt, således vi får en aflæsning hver 1. sekundt. 
      delay(1000);
      ble.println(ReadTemp());
}

//Vi skal nu behandle vores temperaturmåling.
//Vi aflæser den nuværende temperatur, vi bruger float således at vi kan få en værdi der er et decimal tal, i grader celcius.
float ReadTemp()
{
  //Vi sætter startværdien til 0
  float Result = 0;
//Vi aflæser dataen fra vores thermocouple (i 16 bit) for at få den nuværende temperatur.
   digitalWrite(CS_PIN, LOW);
  int Data = shiftIn(SO_PIN, SCK_PIN, MSBFIRST) << 8;
  Data |= shiftIn(SO_PIN, SCK_PIN, MSBFIRST);
  digitalWrite(CS_PIN, HIGH);

  //Vi checker igen for en løs forbindelse
  if(Data & 0x4){
    Result = -1;
  }
  else
  {
Data = Data >> 3;
//Vi vil gerne vise vores data i intervaller af 0.25. (ie. 0,0.25,0.50,0.75,1,0.)
Result = Data * 0.25;
  }

 return Result;

  char n, inputs[BUFSIZE + 1];
   
  if (Serial.available())
  {
    n = Serial.readBytes(inputs, BUFSIZE);
    inputs[n] = 0;
    // Send characters to Bluefruit
    Serial.print("Sending: ");
    Serial.println(inputs);

    // Send input data to host via Bluefruit
    ble.print(inputs);
  }
  if (ble.available()) {
    Serial.print("* "); Serial.print(ble.available()); Serial.println(F(" bytes available from BTLE"));
  }
  // Echo received data
  while ( ble.available() )
  {
    int c = ble.read();
    Serial.print((char)c);
  }
  delay(1000);
}
