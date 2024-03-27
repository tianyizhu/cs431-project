/**
 *  @file   sensor.cpp
 *  @author Simon Yu
 *  @date   01/12/2022
 *  @brief  Sensor class source.
 *
 *  This file implements the sensor class.
 */

/*
 *  External headers.
 */
#include <esp32-hal-gpio.h>

/*
 *  Project headers.
 */
#include "common/global.h"
#include "common/parameter.h"
#include "common/pin.h"
#include "platform/io_expander.h"
#include "sensor/sensor.h"

/*
 *  Biped namespace.
 */
namespace biped
{
/*
 *  Firmware namespace.
 */
namespace firmware
{
Sensor::Sensor()
{
    /*
     *  Using I/O expander global shared pointers and the I/O expander pinModePort
     *  functions, set pin mode for the time-of-flight shutdown pins. Use
     *  pull-up if the pin mode is input.
     *
     *  Learn more about pull-up resistors here:
     *  https://en.wikipedia.org/wiki/Pull-up_resistor
     *
     *  Refer to the global header for the global shared pointers, the I/O expander
     *  header for the pinModePort functions, the pin header for the time-of-flight
     *  shutdown pins, and the esp32-hal-gpio header for the available pin modes
     *  (GPIO FUNCTIONS).
     *
     *  TODO LAB 6 YOUR CODE HERE.
     */

	io_expander_a_->pinModePortA(IOExpanderAPortAPin::time_of_flight_left_shutdown, OUTPUT);
	io_expander_a_->pinModePortB(IOExpanderAPortBPin::time_of_flight_middle_shutdown, OUTPUT);
	io_expander_a_->pinModePortB(IOExpanderAPortBPin::time_of_flight_right_shutdown, OUTPUT);

    /*
     *  Instantiate the class member time-of-flight objects using the C++ STL
     *  std::make_unique function.
     *
     *  Refer to the time-of-flight header for constructor parameters, the
     *  parameter headers for the time-of-flight addresses, the pin header for
     *  the time-of-flight pins.
     *
     *  TODO LAB 6 YOUR CODE HERE.
     */
	time_of_flight_left_ = std::make_unique<TimeOfFlight>(AddressParameter::time_of_flight_left,
	                                                      IOExpanderAPortAPin::time_of_flight_left_shutdown,
	                                                      io_expander_a_);
	time_of_flight_middle_ = std::make_unique<TimeOfFlight>(AddressParameter::time_of_flight_middle,
	                                                        IOExpanderAPortBPin::time_of_flight_middle_shutdown + 8,
	                                                        io_expander_a_);
	time_of_flight_right_ = std::make_unique<TimeOfFlight>(AddressParameter::time_of_flight_right,
	                                                       IOExpanderAPortBPin::time_of_flight_right_shutdown + 8,
	                                                       io_expander_a_);

}

EncoderData
Sensor::getEncoderData() const
{
    /*
     *  Get encoder data struct from the class member encoder
     *  object and return the struct.
     *
     *  TODO LAB 6 YOUR CODE HERE.
     */
    return encoder_.getData();
}

IMUData
Sensor::getIMUData() const
{
    /*
     *  Get IMU data struct from the class member IMU
     *  object and return the struct.
     *
     *  TODO LAB 6 YOUR CODE HERE.
     */
    return imu_.getData();
}

TimeOfFlightData
Sensor::getTimeOfFlightData() const
{
    /*
     *  Return the class member time-of-flight data struct.
     *
     *  TODO LAB 6 YOUR CODE HERE.
     */
    return time_of_flight_data_;
}

void
Sensor::sense(const bool& fast_domain)
{
    /*
     *  Tasks in the fast domain, such as IMU reading and attitude calculation,
     *  require a high frequency of execution. Performing these tasks too
     *  infrequently can lead to delays, resulting in inaccurate or outdated data
     *  samples. For instance, if the attitudes were calculated at sparse intervals,
     *  the Biped's actual attitudes could have changed significantly in the interim.
     *  This delay in data acquisition can adversely affect the Biped's balance, as it
     *  relies on timely and accurate information to maintain the unstable equilibrium.
     *
     *  On the contrary, slow domain tasks are more suited to a lower execution frequency,
     *  often due to factors like sensor noise or performance considerations. For instance,
     *  the calculation of X velocity, which is determined by the number of encoder
     *  steps over time, illustrates this point well. A shorter measurement period results
     *  in noisier velocity data. Furthermore, there's no necessity for frequent readings
     *  of non-essential sensors such as the time-of-flight sensors. Operating these tasks
     *  in the slow domain is more efficient and performance-friendly, balancing accuracy
     *  and system resource utilization.
     */
    if (fast_domain)
    {
        /*
         *  Perform encoder read using the class member encoder object.
         *
         *  Refer to the encoder header for the encoder functions.
         *
         *  TODO LAB 6 YOUR CODE HERE.
         */
    	encoder_.read();

        /*
         *  Perform IMU read using the class member IMU object.
         *
         *  Refer to the IMU header for the IMU functions.
         *
         *  TODO LAB 6 YOUR CODE HERE.
         */
    	imu_.read();
    }
    else
    {
        /*
         *  Calculate velocity using the class member encoder object.
         *
         *  Refer to the encoder header for the encoder functions.
         *
         *  TODO LAB 6 YOUR CODE HERE.
         */
    	encoder_.calculateVelocity();

        /*
         *  Perform time-of-flight reads using the class member
         *  time-of-flight objects, and store the read values
         *  to the class member time-of-flight data struct.
         *
         *  Refer to the time-of-flight header for the time-of-flight
         *  functions, and type header for the time-of-flight data struct.
         *
         *  TODO LAB 6 YOUR CODE HERE.
         */
    	time_of_flight_data_.range_left = time_of_flight_left_->read();
    	time_of_flight_data_.range_middle = time_of_flight_middle_->read();
    	time_of_flight_data_.range_right = time_of_flight_right_->read();
    }
}

void IRAM_ATTR
Sensor::onEncoderLeftA()
{
    /*
     *  Call left encoder A callback function using the class member
     *  encoder object.
     *
     *  Refer to the encoder header for the encoder functions.
     *
     *  TODO LAB 6 YOUR CODE HERE.
     */
	encoder_.onLeftA();
}


void IRAM_ATTR
Sensor::onEncoderLeftB()
{
    /*
     *  Call left encoder B callback function using the class member
     *  encoder object.
     *
     *  Refer to the encoder header for the encoder functions.
     *
     *  TODO LAB 6 YOUR CODE HERE.
     */
	encoder_.onLeftB();
}

void IRAM_ATTR
Sensor::onEncoderRightA()
{
    /*
     *  Call right encoder A callback function using the class member
     *  encoder object.
     *
     *  Refer to the encoder header for the encoder functions.
     *
     *  TODO LAB 6 YOUR CODE HERE.
     */

	encoder_.onRightA();
}

void IRAM_ATTR
Sensor::onEncoderRightB()
{
    /*
     *  Call right encoder B callback function using the class member
     *  encoder object.
     *
     *  Refer to the encoder header for the encoder functions.
     *
     *  TODO LAB 6 YOUR CODE HERE.
     */
	encoder_.onRightB();
}
}   // namespace firmware
}   // namespace biped
