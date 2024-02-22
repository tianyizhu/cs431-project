/**
 *  @file   main.cpp
 *  @author Simon Yu
 *  @date   01/12/2022
 *  @brief  Main program source.
 *
 *  This file implements the main program.
 */

/*
 *  External headers.
 */
#include <EEPROM.h>
#include <esp32-hal-gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <Wire.h>

/*
 *  Project headers.
 */
#include "actuator/actuator.h"
#include "common/global.h"
#include "common/pin.h"
#include "controller/controller.h"
#include "network/udp.h"
#include "platform/camera.h"
#include "platform/display.h"
#include "platform/io_expander.h"
#include "platform/serial.h"
#include "platform/timer.h"
#include "platform/wifi.h"
#include "planner/maneuver_planner.h"
#include "planner/waypoint_planner.h"
#include "sensor/sensor.h"
#include "task/interrupt.h"
#include "task/task.h"
#include "utility/math.h"

/*
 *  Use Biped firmware namespace.
 */
using namespace biped::firmware;

/**
 *  @brief  Arduino setup function.
 *
 *  This function creates, configures, and launches drivers, objects,
 *  and tasks. The function also sets pin modes and attaches interrupt
 *  handlers to the corresponding pins.
 */
void
setup()
{
    /*
     *  Using the Arduino pinMode function, set pin mode for
     *  the I/O expander interrupt pins. Use pull-up if the pin
     *  mode is input.biped
     *
     *  Learn more about pull-up resistors here:
     *  https://en.wikipedia.org/wiki/Pull-up_resistor
     *
     *  Refer to the pin header for the I/O expander interrupt
     *  pins and the esp32-hal-gpio header for the available pin
     *  modes (GPIO FUNCTIONS).
     *
     *  TODO LAB 4 YOUR CODE HERE.
     */

    /*
     *  Set Arduino I2C driver object (Wire) SDA and SCL pins
     *  and set the serial object maximum log level.
     *
     *  Refer to the Arduino Wire header for their functions, the
     *  serial header for the serial functions, and
     *  the parameter header for the I2C SDA and SCL
     *  pins and the maximum log level.
     *
     *  TODO LAB 1 YOUR CODE HERE.
     */
	Wire.setPins(ESP32Pin::i2c_sda, ESP32Pin::i2c_scl);
	Serial::setLogLevelMax(SerialParameter::log_level_max);
//	Serial.begin();


    /*
     *  Initialize the Arduino I2C driver (Wire), the Arduino
     *  EEPROM driver (EEPROM), the display object, and the
     *  serial object.
     *
     *  Refer to the Arduino Wire and EEPROM headers for their
     *  functions, the display and serial headers for their
     *  functions, and the parameters header for the EEPROM size.
     *
     *  TODO LAB 1 YOUR CODE HERE.
     */
	Wire.begin();
	EEPROM.begin(EEPROMParameter::size);
	Display::initialize();
	Serial::initialize();


    /*
     *  Read the Biped serial number from the EEPROM and store it to the
     *  Biped serial number global variable.
     *
     *  Remember to static_cast the read value as unsigned before storing
     *  it to the Biped serial number global variable.
     *
     *  Note that one should always use C++ explicit type casts (static_cast,
     *  dynamic_cast, etc.) when programming in C++ instead of the C-style
     *  type cast, as the C++ type casts are checked by the compiler, whereas
     *  C-style casts are not and can fail at runtime.
     *
     *  Learn more about C++ type casting here:
     *  https://cplusplus.com/doc/oldtutorial/typecasting/
     *
     *  Refer to the global header for the Biped serial number global variable,
     *  the Arduino EEPROM header for reading from EEPROM, and the parameter
     *  header for the Biped serial number EEPROM address.
     *
     *  TODO LAB 1 YOUR CODE HERE.
     */
	serial_number_ = static_cast<unsigned>(EEPROM.readByte(AddressParameter::eeprom_serial_number));

    /*
     *  Instantiate the camera and the NeoPixel global objects using the C++
     *  STL std::make_shared function.
     *
     *  Note that the camera object must be instantiated first before the rest
     *  of the objects.
     *
     *  Refer to the global header for the global object shared pointers.
     *
     *  TODO LAB 1 YOUR CODE HERE.
     */
	camera_ = std::make_shared<Camera>();
	neopixel_ = std::make_shared<NeoPixel>();

    /*
     *  Instantiate the timer global object using the C++ STL
     *  std::make_shared function.
     *
     *  Refer to the global header for the global object shared pointers,
     *  and the parameter headers for the timer group and index.
     *
     *  TODO LAB 2 YOUR CODE HERE.
     */
	 timer_ = std::make_shared<Timer>(TimerParameter::group, TimerParameter::index);

    /*
     *  Instantiate the I/O expander global objects using the C++ STL
     *  std::make_shared function.
     *
     *  Refer to the global header for the global object shared pointers,
     *  and the parameter headers for the I/O expander addresses.
     *
     *  TODO LAB 4 YOUR CODE HERE.
     */

    /*
     *  Instantiate the UDP and Wi-Fi global objects using the C++ STL
     *  std::make_shared function.
     *
     *  Refer to the global header for the global object shared pointers.
     *
     *  TODO LAB 5 YOUR CODE HERE.
     */

    /*
     *  Instantiate the sensor and actuator global objects using the C++ STL
     *  std::make_shared function.
     *
     *  Refer to the global header for the global object shared pointers.
     *
     *  TODO LAB 6 YOUR CODE HERE.
     */

    /*
     *  Instantiate the controller object using the C++ STL std::make_shared
     *  function.
     *
     *  Refer to the global header for the global object shared pointers.
     *
     *  TODO LAB 7 YOUR CODE HERE.
     */

    /*
     *  Using the controller global shared pointer, set the controller periods.
     *
     *  Remember to set both the fast and slow domain periods.
     *
     *  Refer to the global header for the global shared pointers and the parameter
     *  header for the period values.
     *
     *  TODO LAB 7 YOUR CODE HERE.
     */

    /*
     *  Instantiate the planner global object using the C++ STL std::make_shared
     *  function. Note that the planner global shared pointer is a base pointer
     *  to the parent Planner abstract class of the WaypointPlanner and ManeuverPlanner
     *  classes. If one would like to use the WaypointPlanner as the planner, pass the
     *  WaypointPlanner to the template argument of std::make_shared. Otherwise, pass
     *  ManeuverPlanner.
     *
     *  Learn more about C++ Polymorphism here:
     *  https://www.w3schools.com/cpp/cpp_polymorphism.asp
     *biped
     *  Refer to the global header for the global object shared pointers.
     *
     *  TODO LAB 8 YOUR CODE HERE.
     */

    /*
     *  Create I/O expander interrupt service tasks using the FreeRTOS
     *  xTaskCreatePinnedToCore function. Set the task descriptive name to be the
     *  same as the task function name. The I/O expander interrupt tasks have the
     *  highest priority. Pin both tasks to core 1. The parameter pointer for
     *  the task (pvParameters) should be a null pointer.
     *
     *  Note that due to the existence of smart pointers in C++ (unique pointer,
     *  shared pointer, etc.) One should always use the nullptr keyword to denote
     *  all null pointer values when programming in C++, instead of the C macro NULL.
     *
     *  Refer to the task header for the task functions, the global header for the task
     *  handle pointers and the parameter header for the task parameters, such as the
     *  stack size, priority, CPU core, etc.
     *
     *  TODO LAB 3 YOUR CODE HERE.
     */
	 xTaskCreatePinnedToCore(ioExpanderAInterruptServiceTask, "ioExpanderAInterruptServiceTask", TaskParameter::stack_size,
			 nullptr, TaskParameter::priority_max, &task_handle_io_expander_a_interrupt_service_, 1);

	 xTaskCreatePinnedToCore(ioExpanderBInterruptServiceTask, "ioExpanderBInterruptServiceTask", TaskParameter::stack_size,
			 nullptr, TaskParameter::priority_max, &task_handle_io_expander_b_interrupt_service_, 1);

    /*
     *  Using the attachInterrupt function in the interrupt header, attach the I/O
     *  expander interrupt handlers in on-high mode.
     *
     *  Why should the I/O expander interrupt mode be on-high? Consider the following
     *  question. Would the I/O expander interrupt service still be able to handle
     *  any level-triggered interrupts (on-high, on-low, etc.) from the I/O expander pins
     *  if the I/O expander interrupts to the ESP32 below are attached with edge-triggered
     *  interrupt modes (rising, falling, etc.)
     *
     *  Learn more about level-triggered vs. edge-triggered interrupts here:
     *  https://www.garystringham.com/level-triggered-vs-edge-triggered-interrupts/
     *
     *  Note that the attachInterrupt function in the interrupt header has identical
     *  name to the Arduino attachInterrupt function. Since we are currently in the global
     *  namespace and outside Biped firmware namespaces, therefore, to avoid collision,
     *  explicitly resolve the namespaces by calling the attachInterrupt function in the
     *  interrupt header as biped::firmware::attachInterrupt. The explicit namespace
     *  resolution is always necessary for biped::firmware::attachInterrupt since its
     *  function signature is too similar to that of the Arduino attachInterrupt function.
     *
     *  Additionally, always wrap the ESP32 interrupt pins around the digitalPinToInterrupt
     *  macro to ensure that the given pin numbers are within the ESP32 interrupt pin range.
     *
     *  Refer to the interrupt header for the biped::firmware::attachInterrupt
     *  function, the pin header for the I/O expander interrupt pins, and the esp32-hal-gpio
     *  header for the available interrupt modes (Interrupt Modes).
     *
     *  TODO LAB 4 YOUR CODE HERE.
     */

    /*
     *  Using the attachInterrupt function in the interrupt header, attach the encoder
     *  interrupt handlers. What should the interrupt mode for the encoder interrupt
     *  handlers be?
     *
     *  Note that the attachInterrupt function in the interrupt header has identical
     *  name to the Arduino attachInterrupt function. Since we are currently in the global
     *  namespace and outside Biped firmware namespaces, therefore, to avoid collision,
     *  explicitly resolve the namespaces by calling the attachInterrupt function in the
     *  interrupt header as biped::firmware::attachInterrupt. The explicit namespace
     *  resolution is always necessary for biped::firmware::attachInterrupt since its
     *  function signature is too similar to that of the Arduino attachInterrupt function.
     *
     *  Additionally, always wrap the ESP32 interrupt pins around the digitalPinToInterrupt
     *  macro to ensure that the given pin numbers are within the ESP32 interrupt pin range.
     *
     *  Refer to the interrupt header for the biped::firmware::attachInterrupt
     *  function, the pin header for the encoder pins, and the esp32-hal-gpio header for the
     *  available interrupt modes (Interrupt Modes).
     *
     *  TODO LAB 6 YOUR CODE HERE.
     */

    /*
     *  Using I/O expander global shared pointers and the I/O expander pinModePort
     *  functions, set pin mode for the push button pins. Use pull-up if the pin mode
     *  is input.
     *
     *  Learn more about pull-up resistors here:
     *  https://en.wikipedia.org/wiki/Pull-up_resistor
     *
     *  Refer to the global header for the global shared pointers, the I/O expander
     *  header for the pinModePort functions, the pin header for the push button pins
     *  and the esp32-hal-gpio header for the available pin modes (GPIO FUNCTIONS).
     *
     *  TODO LAB 4 YOUR CODE HERE.
     */

    /*
     *  Using I/O expander global shared pointers and the I/O expander attachInterruptPort
     *  functions, attach the push button interrupt handlers. The argument pointer for the
     *  interrupt handler should be a null pointer. Choose the interrupt mode such that the
     *  interrupt is triggered as soon as the push button is pressed down.
     *
     *  Note that due to the existence of smart pointers in C++ (unique pointer,
     *  shared pointer, etc.) One should always use the nullptr keyword to denote
     *  all null pointer values when programming in C++, instead of the C macro NULL.
     *
     *  Additionally, do not wrap the I/O expander pins around the digitalPinToInterrupt
     *  macro since the macro is only for ESP32 pins.
     *
     *  Refer to the global header for the global shared pointers, the I/O expander header
     *  for the attachInterruptPort functions, the pin header for the push button pins, and
     *  the interrupt header for the push button interrupt handlers, and the esp32-hal-gpio
     *  header for the available interrupt modes (Interrupt Modes).
     *
     *  TODO LAB 4 YOUR CODE HERE.
     */

    /*
     *  Create the real-time task, all UDP tasks, and the network task using the
     *  FreeRTOS xTaskCreatePinnedToCore function. Set the task descriptive name to
     *  be the same as the task function name. The real-time task has the second-highest
     *  priority, right below the I/O expander interrupt tasks. The remaining tasks
     *  should have the lowest priority. Pin all tasks to core 1. The parameter pointer
     *  for all tasks (pvParameters) should be a null pointer.
     *
     *  Note that due to the existence of smart pointers in C++ (unique pointer,
     *  shared pointer, etc.) One should always use the nullptr keyword to denote
     *  all null pointer values when programming in C++, instead of the C macro NULL.
     *
     *  Why does the real-time task have a lower priority than the I/O expander interrupt
     *  tasks? Shouldn't the real-time task be the most important task? To answer this
     *  question, think about whether it is appropriate for the I/O expander interrupts
     *  to interrupt the real-time task.
     *
     *  Refer to the task header for the task functions, the global header for the task handle
     *  pointers and the parameter header for the task parameters, such as the stack size,
     *  priority, CPU core, etc.
     *
     *  TODO LAB 3 YOUR CODE HERE.
     */

	 xTaskCreatePinnedToCore(networkTask, "networkTask", TaskParameter::stack_size,
			 nullptr, TaskParameter::priority_min, &task_handle_network_, 1);

	 xTaskCreatePinnedToCore(realTimeTask, "realTimeTask", TaskParameter::stack_size,
			 nullptr, TaskParameter::priority_max-1, &task_handle_real_time_, 1);

	 xTaskCreatePinnedToCore(udpReadBipedMessageTask, "udpReadBipedMessageTask", TaskParameter::stack_size,
			 nullptr, TaskParameter::priority_min, &task_handle_udp_read_biped_message_, 1);

	 xTaskCreatePinnedToCore(udpWriteBipedMessageTask, "udpWriteBipedMessageTask", TaskParameter::stack_size,
			 nullptr, TaskParameter::priority_min, &task_handle_udp_write_biped_message_, 1);

	 xTaskCreatePinnedToCore(udpWriteCameraTask, "udpWriteCameraTask", TaskParameter::stack_size,
			 nullptr, TaskParameter::priority_min, &task_handle_udp_write_camera_, 1);
    /*
     *  Using the timer global shared pointer, set the hardware timer interval to be the fast
     *  domain period. Be aware of the unit conversions and use the appropriate functions
     *  in the math header to convert the units correctly.
     *
     *  Refer to the timer header for the timer functions and the parameter header for the
     *  period values.
     *
     *  TODO LAB 2 YOUR CODE HERE.
     */
	 timer_->setInterval(static_cast<uint64_t>(secondsToMicroseconds(PeriodParameter::fast)));

    /*
     *  Using the timer global shared pointer, attach the timer interrupt handler to the
     *  hardware timer.
     *
     *  Refer to the timer header for the timer functions and the interrupt header for the
     *  interrupt handlers.
     *
     *  TODO LAB 2 YOUR CODE HERE.
     */
     timer_->attachInterrupt(&timerInterruptHandler);

    /*
     *  Using the timer global shared pointer, enable the hardware timer.
     *
     *  Refer to the timer header for the timer functions.
     *
     *  TODO LAB 2 YOUR CODE HERE.
     */
     timer_->enable();


    /*
     *  Print to the serial using the Serial class in the serial header. Pass the desired log
     *  level as the constructor argument, and then use the class member insertion operator (<<)
     *  to stream items (string literals, variables) directly to the constructor call, similar to
     *  using the C++ STL std::cout. Note that a new-line character is automatically appended to
     *  the end by the Serial class if not in raw mode.
     *
     *  Refer to the C++ STL ostream usages here:
     *  https://cplusplus.com/reference/ostream/ostream/operator%3C%3C/
     *
     *  Using the Serial class in the serial header, print the warning message
     *  "Initialized with error(s)." to the serial if the current worst log level is worse than
     *  or equal to error. Otherwise, print the information message "Initialized." to the serial.
     *
     *  Note that the Serial class in the serial header has identical name to the Arduino
     *  Serial object. Since we are currently in the global namespace and outside Biped firmware
     *  namespaces, therefore, to avoid collision, explicitly resolve the namespaces by calling the
     *  Serial class constructor in the serial header as biped::firmware::Serial. The explicit
     *  namespace resolution for the Serial class is only necessary here in the main program
     *  where everything is outside the Biped firmware namespaces.
     *
     *  Refer to the serial header for the serial functions and the type header for the log
     *  levels.
     *
     *  TODO LAB 1 YOUR CODE HERE.
     */
//	biped::firmware::Serial s =
	if (Serial::getLogLevelWorst() <= LogLevel::error)
		biped::firmware::Serial(LogLevel::warn) << "Initialized with error(s): " << (int)Serial::getLogLevelWorst();
	else
		biped::firmware::Serial(LogLevel::info) << "Initialized";

     biped::firmware::Serial(LogLevel::info) << "serial number: " << serial_number_;
}

/**
 *  @brief  Arduino loop function.
 *
 *  This function is called by a FreeRTOS loop task created and
 *  launched by the Arduino board support package (BSP) on top
 *  of the ESP-IDF. The FreeRTOS loop task has a low priority
 *  and the loop function calls the best-effort task function.
 */
void
loop()
{
    /*
     *  Perform best-effort tasks.
     *
     *  Refer to the task header for the task functions.
     *
     *  TODO LAB 1 YOUR CODE HERE.
     */
	bestEffortTask();
}
