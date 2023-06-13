/** =========================================================================
 * @file Test_02_wiper.ino
 * @brief An example using only using ClariVUE10 turbidity sensor and no logging.
 *        This example is used to test the wiper function of the sensor.
 *        The sensor is powered on and the wiper is activated for 10 seconds.
 *        The sensor is then powered off.
 *  
 * @author Adolfo Lopez Miranda
 *
 * Build Environment: Visual Studios Code with PlatformIO
 * Hardware Platform: EnviroDIY Mayfly Arduino Datalogger
 *
 * DISCLAIMER:
 * THIS CODE IS PROVIDED "AS IS" - NO WARRANTY IS GIVEN.
 * ======================================================================= */

// ==========================================================================
// Include the base required libraries
// ==========================================================================
/** Start [includes] */
// The Arduino library is needed for every Arduino program.
#include <Arduino.h>

// EnableInterrupt is used by ModularSensors for external and pin change
// interrupts and must be explicitely included in the main program.
#include <EnableInterrupt.h>

// Include the main header for ModularSensors
#include <ModularSensors.h>
/** End [includes] */

// ==========================================================================
//  Data Logging Options
// ==========================================================================
/** Start [logging_options] */
// The name of this program file
const char* sketchName = "Test_02_wiper.ino";

// Set the input and output pins for the logger
// NOTE:  Use -1 for pins that do not apply
const int32_t serialBaud = 115200;  // Baud rate for debugging
const int8_t  greenLED   = 8;       // Pin for the green LED
const int8_t  redLED     = 9;       // Pin for the red LED
const int8_t sensorPowerPin = 22;  // MCU pin controlling main sensor power
// Pins for wiper control module
const int8_t wiperPin = 10; // The data pin on Mayfly grove
/** End [logging_options] */


// ==========================================================================
//  Using the Processor as a Sensor
// ==========================================================================
/** Start [processor_sensor] */
#include <sensors/ProcessorStats.h>

// Create the main processor chip "sensor" - for general metadata
const char*    mcuBoardVersion = "v1.1";
ProcessorStats mcuBoard(mcuBoardVersion);
/** End [processor_sensor] */


// ==========================================================================
//  Campbell ClariVUE Turbidity Sensor
// ==========================================================================
/** Start [clarivue10] */
#include <sensors/CampbellClariVUE10.h>

// NOTE: Use -1 for any pins that don't apply or aren't being used.
const char* ClariVUESDI12address = "0";  // The SDI-12 Address of the ClariVUE10
const int8_t ClariVUEPower       = sensorPowerPin;  // Power pin
const int8_t ClariVUEData        = 7;               // The SDI-12 data pin
// NOTE:  you should NOT take more than one readings.  THe sensor already takes
// and averages 8 by default.

// Create a Campbell ClariVUE10 sensor object
CampbellClariVUE10 clarivue(*ClariVUESDI12address, ClariVUEPower, ClariVUEData);
/** End [clarivue] */


// ==========================================================================
//  Creating the Variable Array[s] and Filling with Variable Objects
// ==========================================================================
/** Start [variable_arrays] */
Variable* variableList[] = {
    new ProcessorStats_SampleNumber(&mcuBoard),
    new ProcessorStats_FreeRam(&mcuBoard),
    new ProcessorStats_Battery(&mcuBoard),
    new CampbellClariVUE10_Turbidity(&clarivue),
    new CampbellClariVUE10_Temp(&clarivue),
    new CampbellClariVUE10_ErrorCode(&clarivue)
};
// Count up the number of pointers in the array
int variableCount = sizeof(variableList) / sizeof(variableList[0]);

// Create the VariableArray object
VariableArray varArray;
/** End [variable_arrays] */


// ==========================================================================
//  The Logger Object[s]
// ==========================================================================
/** Start [loggers] */
// Create a logger instance
Logger dataLogger;
/** End [loggers] */


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

// ZebraTech Hydro-Wiper turns on the wiper to clean the optical lens from the sensor
void wiperController() {
    // Turn on the LED to show we're turning on the wiper controller and take a reading
    digitalWrite(greenLED, HIGH);

    // Turn on the Wiper Controller on Hydro-Wiper
    // Note ZebraTech Hydro-Wiper requires the control line to be set
    // high to +5 volts for a minimum of 10ms
    Serial.println("Turning on Wiper Controller");
    digitalWrite(wiperPin, HIGH);
    delay(100); // wait 100ms before turning off the control line.

    // Turn off the Wiper Controller
    Serial.println("Turning off Wiper Controller");
    digitalWrite(wiperPin, LOW);

    // Wait for the wipe to finish
    Serial.println("Waiting for Wiper to finish");
    delay(10000); //wiper takes about 10 seconds
    digitalWrite(greenLED, LOW);
    // Note: Wiper takes closer to 5-6 seconds to finsih when not submerged in water
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

    Serial.print(F("Using ModularSensors Library version "));
    Serial.println(MODULAR_SENSORS_VERSION);

    // Set up pins for the LED's
    pinMode(greenLED, OUTPUT);
    pinMode(redLED, OUTPUT);
    // Blink the LEDs to show the board is on and starting up
    greenredflash();

    // Set up pins for wiper control module 
    pinMode(wiperPin, OUTPUT);

    // Begin the variable array[s], logger[s], and publisher[s]
    varArray.begin(variableCount, variableList);

    // Set up the sensors
    Serial.println(F("Setting up sensors..."));
    varArray.setupSensors();
}
/** End [setup] */


// ==========================================================================
//  Arduino Loop Function
// ==========================================================================
/** Start [loop] */
void loop() {
    // Turn on the LED and wipe the optical lens from the sensor
    wiperController();

    // Turn on the LED to show we're taking a reading
    digitalWrite(greenLED, HIGH);

    // Send power to the sensor
    varArray.sensorsPowerUp();

    // Wake up the sensor
    varArray.sensorsWake();

    // Update the sensor value
    varArray.updateAllSensors();

    // Print the sensor measurements
    Serial.print("Current sensor measurements range: ");
    varArray.printSensorData();


    // Put the sensor back to sleep
    varArray.sensorsSleep();

    // Cut the sensor power
    varArray.sensorsPowerDown();

    // Turn off the LED to show we're done with the reading
    digitalWrite(greenLED, LOW);

    // Wait for the next reading
    delay(5000);
}
/** End [loop] */