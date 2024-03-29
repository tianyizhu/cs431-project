/**
 *  @file   task.h
 *  @author Simon Yu
 *  @date   12/03/2022
 *  @brief  Task function header.
 *
 *  This file defines the task functions.
 */

/*
 *  Include guard.
 */
#ifndef TASK_TASK_H_
#define TASK_TASK_H_

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
 *  @brief  Best-effort task function.
 *
 *  This function performs the best-effort tasks that
 *  do not require strict timing deadlines. The function
 *  is called by the main program Arduino loop function.
 *  The best-effort task has a low priority and may be
 *  preempted by any tasks with the same or higher
 *  priorities or by priority inheritance.
 */
void
bestEffortTask();

/**
 *  @param  pvParameters Function argument pointer.
 *  @brief  I/O expander A interrupt service task function.
 *
 *  This function handles interrupts to ESP32 generated by
 *  the I/O expander A. The task has the highest priority
 *  but may be preempted by any tasks with the same priority
 *  or by priority inheritance. The function goes to sleep
 *  until woken.
 */
void
ioExpanderAInterruptServiceTask(void* pvParameters);

/**
 *  @param  pvParameters Function argument pointer.
 *  @brief  I/O expander B interrupt service task function.
 *
 *  This function handles interrupts to ESP32 generated by
 *  the I/O expander B. The task has the highest priority
 *  but may be preempted by any tasks with the same priority
 *  or by priority inheritance. The function goes to sleep
 *  until woken.
 */
void
ioExpanderBInterruptServiceTask(void* pvParameters);

/**
 *  @param  pvParameters Function argument pointer.
 *  @brief  Network task function.
 *
 *  This function initializes the network drivers and the
 *  UDP interfaces. The network task has the lowest priority
 *  and may be preempted by any tasks with the same or higher
 *  priorities.
 */
void
networkTask(void* pvParameters);

/**
 *  @param  pvParameters Function argument pointer.
 *  @brief  Real-time task function.
 *
 *  This function performs the real-time task with strict timing
 *  deadlines. There are two timing domains, a fast domain and
 *  a slow domain. The real-time task has a high priority but
 *  may be preempted by any tasks with the same or higher
 *  priorities or by priority inheritance. The function goes to
 *  sleep until woken.
 *
 *  This function periodically executes the Biped software stack,
 *  comprised of three primary stages: sensing, control, and
 *  actuation. The stack first performs sensing, where it collects
 *  data from various sensors. Then, the stack executes the controller
 *  which takes the collected sensor data and produces an actuation
 *  command. Finally, in the actuation stage, the actuator object
 *  receives the actuation command and performs the actuation.
 */
void
realTimeTask(void* pvParameters);

/**
 *  @param  pvParameters Function argument pointer.
 *  @brief  Biped message UDP read task function.
 *
 *  This function reads the UDP and deserializes the messages read into
 *  the Biped message struct. The Biped message UDP read task has the
 *  lowest priority and may be preempted by any tasks with the same or
 *  higher priorities.
 */
void
udpReadBipedMessageTask(void* pvParameters);

/**
 *  @param  pvParameters Function argument pointer.
 *  @brief  Biped message UDP write task function.
 *
 *  This function serializes the Biped message struct and writes serialized
 *  Biped message struct to the UDP. The Biped message UDP write task has
 *  the lowest priority and may be preempted by any tasks with the same or
 *  higher priorities.
 */
void
udpWriteBipedMessageTask(void* pvParameters);

/**
 *  @param  pvParameters Function argument pointer.
 *  @brief  Camera UDP write task function.
 *
 *  This function serializes camera frames and writes serialized camera
 *  frames to the UDP. The camera UDP write task has the lowest priority
 *  and may be preempted by any tasks with the same or higher priorities.
 */
void
udpWriteCameraTask(void* pvParameters);
}   // namespace firmware
}   // namespace biped

#endif  // TASK_TASK_H_
