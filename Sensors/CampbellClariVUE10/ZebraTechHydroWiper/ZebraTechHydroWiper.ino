/** =========================================================================
 * @file ZebraTechHydroWiper.ino
 * @brief An example for using ZebraTech Hydro-Wiper with Mayfly/Arduino and external power source (12V).
 *
 * @author Adolfo Lopez Miranda
 *
 * Build Environment: Visual Studios Code with PlatformIO
 * Hardware Platform: EnviroDIY Mayfly Arduino Datalogger
 *
 * DISCLAIMER:
 * THIS CODE IS PROVIDED "AS IS" - NO WARRANTY IS GIVEN.
 * ======================================================================= */

/** Start [includes] */
// The Arduino library is needed for every Arduino program.
#include <Arduino.h>

// EnableInterrupt is used by ModularSensors for external and pin change
// interrupts and must be explicitely included in the main program.
#include <EnableInterrupt.h>

// Include the main header for ModularSensors
#include <ModularSensors.h>
/** End [includes] */


// The name of this program file
const char* sketchName = "ZebraTechHydroWiper.ino";

// Set the input and output pins for the logger
// NOTE:  Use -1 for pins that do not apply
const int32_t serialBaud = 115200;  // Baud rate for debugging
const int8_t  greenLED   = 8;       // Pin for the green LED
const int8_t  redLED     = 9;       // Pin for the red LED

// Variables for wiper control module
const int8_t wiperController = 10; // The data pin on Mayfly grove

// ==========================================================================
//  Working Functions
// ==========================================================================
/** Start [working_functions] */
// Flashes the LED's on the primary board
void greenredflash(uint8_t numFlash = 4, uint8_t rate = 75) {
    for (uint8_t i = 0; i < numFlash; i++) {
        digitalWrite(greenLED, HIGH);
        digitalWrite(redLED, LOW);
        delay(rate);
        digitalWrite(greenLED, LOW);
        digitalWrite(redLED, HIGH);
        delay(rate);
    }
    digitalWrite(redLED, LOW);
}
/** End [working_functions] */

// ==========================================================================
//  Arduino Setup Function
// ==========================================================================
/** Start [setup] */
void setup() {
    // Start the primary serial connection
    Serial.begin(serialBaud);

    // Print a start-up note to the first serial port
    Serial.print(F("Now running "));
    Serial.println(sketchName);

    // Set up pins for the LED's
    pinMode(greenLED, OUTPUT);
    pinMode(redLED, OUTPUT);
    // Blink the LEDs to show the board is on and starting up
    greenredflash();

    // Set up pins for wiper control module 
    pinMode(wiperController, OUTPUT);
}
/** End [setup] */


// ==========================================================================
//  Arduino Loop Function
// ==========================================================================
/** Start [loop] */
void loop() {
    // Turn on the LED to show we're turning on the wiper controller
    digitalWrite(greenLED, HIGH);

    // Turn on the Wiper Controller on Hydro-Wiper
    // Note ZebraTech Hydro-Wiper requires the control line to be set
    // high to +5 volts for a minimum of 10ms
    Serial.println("Turning on Wiper Controller");
    digitalWrite(wiperController, HIGH);
    delay(100); // wait 100ms before turning off the control line.

    // Turn off the LED and Wiper Controller
    digitalWrite(greenLED, LOW);
    Serial.println("Turning off Wiper Controller");
    digitalWrite(wiperController, LOW);

    // Wait for the wipe to finish
    Serial.println("Waiting for Wiper to finish");
    delay(10000); //wiper takes about 10 seconds
    //Note: Wiper takes closer to 5-6 seconds to finsih when not submerged in water
}
/** End [loop] */