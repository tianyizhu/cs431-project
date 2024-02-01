/**
 *  @file   encoder.h
 *  @author Simon Yu
 *  @date   01/05/2023
 *  @brief  Encoder class header.
 *
 *  This file defines the encoder class.
 */

/*
 *  Include guard.
 */
#ifndef PLATFORM_ENCODER_H_
#define PLATFORM_ENCODER_H_

/*
 *  External headers.
 */
#include <esp_attr.h>

/*
 *  Project headers.
 */
#include "common/type.h"
#include "utility/low_pass_filter.hpp"

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
/**
 *  @brief  Encoder class.
 *
 *  This class provides functions for reading from the motor encoders.
 *  The class also provides callback functions for the interrupts
 *  generated by the encoders, as well as functions for calculating
 *  the linear velocity.
 */
class Encoder
{
public:

    /**
     *  @brief  Encoder class constructor.
     *
     *  This constructor initializes all class member variables.
     *  Additionally, the constructor sets the related I/O pin
     *  modes and configures the low-pass filters.
     */
    Encoder();

    /**
     *  @return Encoder data struct.
     *  @brief  Get the class member encoder data struct.
     *
     *  This function returns the class member encoder data struct.
     */
    EncoderData
    getData() const;

    /**
     *  @brief  Encoder reading function.
     *
     *  This function reads and converts data from class member
     *  encoder step variables set by the encoder interrupt
     *  handlers and populates the corresponding entries in the
     *  class member encoder data struct.
     */
    void
    read();

    /**
     *  @brief  Velocity calculation function.
     *
     *  This function calculates linear velocity from encoder
     *  data and populates the corresponding entries in the member
     *  encoder data struct. The function also filters the calculated
     *  data using the low-pass filter.
     */
    void
    calculateVelocity();

    /**
     *  @brief  Left encoder A callback function.
     *
     *  This function processes the interrupt from the left motor encoder A.
     */
    void IRAM_ATTR
    onLeftA();

    /**
     *  @brief  Left encoder B callback function.
     *
     *  This function processes the interrupt from the left motor encoder B.
     */
    void IRAM_ATTR
    onLeftB();

    /**
     *  @brief  Right encoder A callback function.
     *
     *  This function processes the interrupt from the right motor encoder A.
     */
    void IRAM_ATTR
    onRightA();

    /**
     *  @brief  Right encoder B callback function.
     *
     *  This function processes the interrupt from the right motor encoder B.
     */
    void IRAM_ATTR
    onRightB();

private:

    EncoderData data_;  //!< Encoder data struct.
    LowPassFilter<double> low_pass_filter_velocity_x_;  //!< X velocity low-pass filter object.
    long steps_left_;   //!< Left encoder steps.
    long steps_right_;  //!< Right encoder steps.
};
}   // namespace firmware
}   // namespace biped

#endif  // PLATFORM_ENCODER_H_
