// Copyright Pololu Corporation.  For more information, see http://www.pololu.com/

/** \file Zumo32U4ProximitySensors.h */

#pragma once

#include <stdint.h>
#include <string.h>

/** \brief The pin number for the standard pin that is used to read the left
 * proximity sensor. */
static const uint8_t SENSOR_LEFT = 20;

/** \brief The pin number for the standard pin that is used to read the front
 * proximity sensor. */
static const uint8_t SENSOR_FRONT = 22;

/** \brief The pin number for the standard pin that is used to read the right
 * proximity sensor. */
static const uint8_t SENSOR_RIGHT = 4;

/** \brief A constant that can be used in place of a pin number to indicate that
 * no pin should be used. */
static const uint8_t SENSOR_NO_PIN = 255;

/** \brief Gets readings from the three proximity sensors on the front sensor
 * array.
 *
 * This class allows you to get measurements from the IR proximity sensors on
 * the Zumo 32U4 Front Sensor Array.
 *
 * By default, this class uses pins 20 (A2), 22 (A4), 4, and 11.
 *
 * Since this class uses Zumo32U4IRPulses, which uses Timer 3, it might
 * conflict with other libraries using Timer 3.  Timer 3 is only used while the
 * read() function is running and can be used for other purposes after the
 * function returns.
 *
 * Configuring the pins
 * ====
 *
 * This class allows you to choose what pins will be used for sensors.  Each
 * sensor pin will be configured as a pulled-up input and digital readings will
 * be performed on the pin to see if the sensor is active.
 *
 * By default, this class will drive pin 11 low before taking readings in
 * order to disable the IR emitters of the line sensors so they do not interfere
 * with the proximity sensors.  You can change what pin is used, or you can
 * disable this feature.
 *
 * For more information, see initThreeSensors(), initFrontSensor(), and init().
 *
 * Reading the sensors
 * ===
 *
 * The read() function takes care of disabling the emitters for the line
 * sensors, sending sequences of pulses to the left and right IR LEDs, and
 * performing digital readings on the proximity sensor pins to see if they are
 * active.
 *
 * There are several configuration options that allow you to control the
 * details of how the read() function behaves:
 *
 * * setPeriod()
 * * setBrightnessLevels()
 * * setPulseOnTimeUs()
 * * setPulseOffTimeUs()
 *
 * Interpreting the sensor readings
 * ====
 *
 * The readings generated by the read() function consist of two numbers for each
 * sensor: the number of brightness levels for the left LEDs that activated the
 * sensor, and the number of brightness levels for the right LEDs that activated
 * the sensor.
 *
 * A higher reading corresponds to more IR light getting reflected to the
 * sensor, which is influenced by the size, reflectivity, proximity, and
 * location of nearby objects.  However, the presence of other sources of 38 kHz
 * IR pulses (e.g. from another robot) can also affect the readings.
 *
 * Basic readings
 * ===
 *
 * The readBasic() function does a quick digital reading of a sensor without
 * emitting any IR pulses.  This can be useful for detecting other robots that
 * are emitting pulses of IR at 38 kHz.
 *
 * Brightness levels
 * ===
 *
 * The sequence of IR pulse brightness levels used by the read() function has a
 * big effect on the readings returned by this class.  The levels can be
 * changed by calling setBrightnessLevels().
 *
 * The default IR pulse brightness levels used are 4, 15, 32, 55, 85, and 120.
 *
 * These numbers each represent the pulse width to be used in a burst of IR pulses.
 * Specifically, the pulse width is (1 + brightness) / (16 MHz).
 *
 * We determined by experimenting that the useful range of levels is from about
 * 4 to 120, so we chose those as the minimum and maximum.  We used
 * ((2.236 + 1.756*i)^2 - 1) with i from 0 to 5 to generate the intermediate
 * points. */
class Zumo32U4ProximitySensors
{
public:

    /** \brief The default line sensor emitter pin.
     *
     * This is equal to the SENSOR_LEDON constant defined in
     * Zumo32U4LineSensors.h. */
    static const uint8_t defaultLineSensorEmitterPin = 11;

    /** \brief Minimal constructor.
     *
     * If you use this (i.e. by not providing any arguments when you create the
     * Zumo32U4ProximitySensors object), then you will have to call
     * initThreeSensors(), initFrontSensor(), or init() before using the
     * functions in this class. */
    Zumo32U4ProximitySensors()
    {
        clearAll();
    }

    /** \brief Constructor that takes pin arguments.
     *
     * This constructor calls init() with the specified arguments. */
    Zumo32U4ProximitySensors(uint8_t * pins, uint8_t numSensors,
        uint8_t lineSensorEmitterPin = defaultLineSensorEmitterPin)
    {
        clearAll();
        init(pins, numSensors, lineSensorEmitterPin);
    }

    /** \brief Configures this object to use all three proximity sensors.
     *
     * This function sets up this object to use the left, front, and right
     * proximity sensors on the Zumo32U4 Front Sensor Array.  The pins used
     * for these sensors will be 20 (A2), 22 (A4), and 4 respectively.
     *
     * For this configuration to work, jumpers on the front sensor array must be
     * installed in order to connect pin 20 to LFT and connect pin 4 to RGT.
     *
     * \param lineSensorEmitterPin The number of the pin that controls the
     * emitters for the line sensors.  This pin is used to turn off the emitters
     * for line sensors so they do not cause false readings on the proximity
     * sensors.  You can specify a value of SENSOR_NO_PIN for this parameter if
     * you want this class to not do anything to the emitters. */
    void initThreeSensors(uint8_t lineSensorEmitterPin = defaultLineSensorEmitterPin)
    {
        uint8_t defaultPins[] = { SENSOR_LEFT, SENSOR_FRONT, SENSOR_RIGHT };
        init(defaultPins, sizeof(defaultPins), lineSensorEmitterPin);
    }

    /** \brief Configures this object to use just the front proximity sensor.
     *
     * This function sets up this object to use just the front proximity sensor
     * on the Zumo32U4 Front Sensor Array.  The pin used will be 22 (A4),
     * and the pins for other two sensors will not be modified.
     *
     * \param lineSensorEmitterPin The number of the pin that controls the
     * emitters for the line sensors.  This pin is used to turn off the emitters
     * for line sensors so they do not cause false readings on the proximity
     * sensors.  You can specify a value of SENSOR_NO_PIN for this parameter if
     * you want this class to not do anything to the emitters. */
    void initFrontSensor(uint8_t lineSensorEmitterPin = defaultLineSensorEmitterPin)
    {
        uint8_t pins[] = { SENSOR_FRONT };
        init(pins, sizeof(pins), lineSensorEmitterPin);
    }

    /** \brief Configures this object to use a custom set of pins.
     *
     * \param pins A pointer to an array with the pin numbers for the sensors.
     * \param numSensors The number of sensors.
     * \param lineSensorEmitterPin The number of the pin that controls the
     * emitters for the line sensors.  This pin is used to turn off the emitters
     * for line sensors so they do not cause false readings on the proximity
     * sensors.  You can specify a value of SENSOR_NO_PIN for this parameter if
     * you want this class to not do anything to the emitters. */
    void init(uint8_t * pins, uint8_t numSensors,
        uint8_t lineSensorEmitterPin = defaultLineSensorEmitterPin);

    /** \brief Returns the number of sensors.
     *
     * This could be useful for iterating through all the sensors. */
    uint8_t getNumSensors() const
    {
        return numSensors;
    }

    /* LED parameters **********************************************************/

    /** \brief The default period for the infrared pulses.
     *
     * The default period is 420, which results in a frequency of 38.005 kHz.
     *
     * The period can be changed by calling setPeriod(). */
    static const uint16_t defaultPeriod = 420;

    /** \brief The default duration of the bursts of infrared pulses emitted, in
     * microseconds.
     *
     * According to the TSSP77038 datasheet, the delay between the start of the
     * IR pulses and the start of the sensor output pulse could be anywhere
     * between 7/(38 kHz) and 15/(38 kHz).
     *
     * The default pulse on time of 16/(38 kHz) = 421 us guarantees we are not
     * missing output pulses by reading the sensor too soon.
     *
     * The on time can be changed by calling setPulseOnTimeUs(). */
    static const uint16_t defaultPulseOnTimeUs = 421;

    /** \brief The default time to leave the infrared LEDs off between readings,
     * in microseconds.
     *
     * Ideally we would like the different measurements taken by the read()
     * function to each be independent and not affect eachother, so we would
     * like the sensor to go back to its original state before we start the next
     * set of pulses.  Therefore, it is necessary to wait long enough to
     * guarantee that the previous IR pulses are no longer affecting the sensor
     * output.
     *
     * According to the TSSP77038 datasheet, the sensor output pulse duration
     * could be up to 6/(38 kHz) longer than the duration of the IR pulses,
     * and the sensor output pulse could start as late as 15/(38 kHz) after
     * the IR pulses start.  Therefore, it is possible for the sensor output
     * pulse to end up to 21/(38 kHz) after the ending of the IR pulses.
     *
     * So the default off time is 22/(38 kHz) = 578 us.
     *
     * In our experiments, we saw that the sensor output pulse actually ends
     * within 300 microseconds after the IR pulses end, so an off time of
     * 300 us might be okay.
     *
     * The off time can be changed by calling setPulseOffTimeUs(). */
    static const uint16_t defaultPulseOffTimeUs = 578*5;

    /** \brief Sets the period used for the IR pulses.
     *
     * The period determines the frequency of the IR pulses, which affects how
     * sensitive the IR proximity sensors are.  The default period results in a
     * frequency of about 38 kHz, which maximizes the sensitivity.
     *
     * This parameter is used as the \c period parameter for
     * Zumo32U4IRPulses::start, so see the documentation of that function for
     * details.
     *
     * \sa defaultPeriod */
    void setPeriod(uint16_t period)
    {
        this->period = period;
    }

    /** \brief Sets the sequence of brightness levels used by read().
     *
     * Each brightness level in the sequence will be used as the \c brightness
     * parameter to Zumo32U4IRPulses::start().
     *
     * Note that the order of the brightness levels does matter because the
     * current-limiting components for the IR LEDs on the Zumo 32U4 Main Board
     * include a filter that causes the IR LED power voltage to decrease
     * gradually while the LEDs are on, and recover gradually while they are
     * off.  With the default timing parameters, the voltage does not recover
     * completely between bursts of IR pulses.
     *
     * \param levels A pointer to an array of brightness levels.
     * \param levelCount The number of brightness levels.
     *
     * \sa defaultBrightnessLevels */
    void setBrightnessLevels(uint16_t * levels, uint8_t levelCount);

    /** \brief Sets the duration, in microseconds, for each burst of IR pulses
     * emitted by the read() function.
     *
     * \sa defaultPulseOnTimeUs */
    void setPulseOnTimeUs(uint16_t pulseOnTimeUs)
    {
        this->pulseOnTimeUs = pulseOnTimeUs;
    }

    /** \brief Sets the amount of time, in microseconds, that the read()
     * function will leave the pulses off before going on to the next step.
     *
     * This delay is also used by lineSensorEmittersOff().
     *
     * \sa defaultPulseOffTimeUs */
    void setPulseOffTimeUs(uint16_t pulseOffTimeUs)
    {
        this->pulseOffTimeUs = pulseOffTimeUs;
    }

    /** \brief Returns the number of brightness levels.
     *
     * This could be useful for formatting the sensor readings for display. */
    uint8_t getNumBrightnessLevels() const
    {
        return numLevels;
    }

    /* Basic operations *********************************************************/

    /** \brief Turns the IR emitters for the line sensors off.
     *
     * Turns the line sensors for the IR emitters off, and then delays.  The pin
     * used to control the emitters is specified in the constructor, or a call
     * to init(), initThreeSensors(), or initFrontSensor().  The pin will be
     * driven low.  The delay can be specified by calling
     * setPulseOffTimeUs().  */
    void lineSensorEmittersOff();

    /** \brief Sets each sensor pin to an input with pull-up resistors enabled. */
    void pullupsOn();

    /** \brief Does a quick digital reading of the specified sensor without emitting
     * any IR pulses.
     *
     * Before calling this function, you should make sure that pullupsOn() and
     * lineSensorEmittersOff() have both been called or else you could get false
     * readings.  You can either call these functions directly or just call
     * read().
     *
     * \return 1 if the sensor is active, 0 if not.
     *
     * \param sensorNumber The zero-based index of the sensor.  This number
     *  should be less than the number of sensors, or else this function's
     *  behavior is undefined. */
    bool readBasic(uint8_t sensorNumber);

    /* Sensor reading ***********************************************************/

    /** \brief Emits IR pulses and gets readings from the sensors.
     *
     * This is the main function of this class.  Almost all other functions in
     * this class serve to configure how this function will work or to access
     * the results from this function.
     *
     * This function performs the following steps:
     *
     * 1. It calls pullupsOn().
     * 2. It calls lineSensorEmittersOff().
     * 3. For each configured brightness level, it:
     *     1. Starts IR pulses on the left LEDs.
     *     2. Takes a reading of each sensor.
     *     3. Turns off the pulses.
     *     4. Starts IR pulses on right LEDs.
     *     5. Takes a reading of each sensor.
     *     6. Turns off the pulses.
     *
     * The brightness levels can be configured with setBrightnessLevels().  The
     * frequency of the IR pulses (which is normally 38 kHz) can be adjusted
     * with setPeriod().  The delays used during this process can be configured
     * with setPulseOnTimeUs() and setPulseOffTimeUs().
     *
     * The output of this function is two numbers for each sensor: the number of
     * brightness levels for left LEDs that activated the sensor, and the number
     * of brightness levels for the right LEDs that activated the sensor.
     *
     * You can retrieve the numbers by calling countsWithLeftLeds(),
     * countsWithRightLeds(), or a number of other helper functions defined in
     * this class.
     *
     * With the default timing parameters, the amount of time this function
     * takes to run is approximately 2.15 milliseconds per brightness level plus
     * 0.62 milliseconds.  The number of sensors has only a small affect on the
     * run time. */
    void read();

    /** \brief Returns the number of brightness levels for the left LEDs that
     * activated the specified sensor.
     *
     * \param sensorNumber The zero-based index of the sensor.  This number
     *  should be less than the number of sensors, or else this function returns
     *  0. */
    uint8_t countsWithLeftLeds(uint8_t sensorNumber) const;

    /** \brief Returns the number of brightness levels for the right LEDs that
     * activated the specified sensor.
     *
     * \param sensorNumber The zero-based index of the sensor.  This number
     *  should be less than the number of sensors, or else this function returns
     *  0. */
    uint8_t countsWithRightLeds(uint8_t sensorNumber) const;

    /* Helper methods for the typical Zumo sensor setup ************************/

    /** \brief Returns the number of brightness levels for the left LEDs that
     * activated the left proximity sensor.
     *
     * This function assumes pin 20 (A2) is connected to the left sensor.
     * It returns 0 if the object is not configured to use that pin as a
     * sensor. */
    uint8_t countsLeftWithLeftLeds() const
    {
        return countsWithLeftLeds(findIndexForPin(SENSOR_LEFT));
    }

    /** \brief Returns the number of brightness levels for the right LEDs that
     * activated the left proximity sensor.
     *
     * This function assumes pin 20 (A2) is connected to the left sensor.
     * It returns 0 if the object is not configured to use that pin as a
     * sensor. */
    uint8_t countsLeftWithRightLeds() const
    {
        return countsWithRightLeds(findIndexForPin(SENSOR_LEFT));
    }

    /** \brief Returns the number of brightness levels for the left LEDs that
     * activated the front proximity sensor.
     *
     * This function assumes pin 22 (A4) is connected to the front sensor.
     * It returns 0 if the object is not configured to use that pin as a
     * sensor. */
    uint8_t countsFrontWithLeftLeds() const
    {
        return countsWithLeftLeds(findIndexForPin(SENSOR_FRONT));
    }

    /** \brief Returns the number of brightness levels for the right LEDs that
     * activated the front proximity sensor.
     *
     * This function assumes pin 22 (A4) is connected to the front sensor.
     * It returns 0 if the object is not configured to use that pin as a
     * sensor. */
    uint8_t countsFrontWithRightLeds() const
    {
        return countsWithRightLeds(findIndexForPin(SENSOR_FRONT));
    }

    /** \brief Returns the number of brightness levels for the left LEDs that
     * activated the right proximity sensor.
     *
     * This function assumes pin 4 is connected to the right sensor.
     * It returns 0 if the object is not configured to use that pin as a
     * sensor. */
    uint8_t countsRightWithLeftLeds() const
    {
        return countsWithLeftLeds(findIndexForPin(SENSOR_RIGHT));
    }

    /** \brief Returns the number of brightness levels for the right LEDs that
     * activated the right proximity sensor.
     *
     * This function assumes pin 4 is connected to the right sensor.
     * It returns 0 if the object is not configured to use that pin as a
     * sensor. */
    uint8_t countsRightWithRightLeds() const
    {
        return countsWithRightLeds(findIndexForPin(SENSOR_RIGHT));
    }

    /** \brief Does a quick digital reading of the left sensor.
     *
     * This function assumes pin 20 (A2) is connected to the left sensor.
     * It returns 0 if the object is not configured to use that pin as a
     * sensor.
     *
     * This function calls readBasic(), so see that function's documentation for
     * more information. */
    bool readBasicLeft()
    {
        return readBasic(findIndexForPin(SENSOR_LEFT));
    }

    /** \brief Does a quick digital reading of the front sensor.
     *
     * This function assumes pin 22 (A4) is connected to the left sensor.
     * It returns 0 if the object is not configured to use that pin as a
     * sensor.
     *
     * This function calls readBasic(), so see that function's documentation for
     * more information. */
    bool readBasicFront()
    {
        return readBasic(findIndexForPin(SENSOR_FRONT));
    }

    /** \brief Does a quick digital reading of the right sensor.
     *
     * This function assumes pin 4 is connected to the right sensor.
     * It returns 0 if the object is not configured to use that pin as a
     * sensor.
     *
     * This function calls readBasic(), so see that function's documentation for
     * more information. */
    bool readBasicRight()
    {
        return readBasic(findIndexForPin(SENSOR_RIGHT));
    }

private:

    void clearAll();
    void prepareToRead();
    uint8_t findIndexForPin(uint8_t pin) const;

    typedef struct SensorData
    {
        uint8_t pin;
        uint8_t withLeftLeds;
        uint8_t withRightLeds;
    } SensorData;

    SensorData * dataArray;
    uint8_t numSensors;

    uint8_t lineSensorEmitterPin;

    uint16_t * levelsArray;
    uint8_t numLevels;

    uint16_t period;
    uint16_t pulseOnTimeUs;
    uint16_t pulseOffTimeUs;
};
