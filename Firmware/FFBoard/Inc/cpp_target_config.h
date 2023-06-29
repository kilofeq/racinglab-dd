/*
 * cpp_target_config.h
 *
 *  Created on: 27.12.2020
 *      Author: willson556
 */

#ifndef CPP_TARGET_CONFIG_H_
#define CPP_TARGET_CONFIG_H_

#include "MotorPWM.h"
#include "target_constants.h"
#include "CAN.h"


#ifdef CANBUS
extern CANPort canport;
#endif

#ifdef PWMDRIVER
extern const PWMConfig pwmTimerConfig;
#endif

#ifdef DEBUGPIN
extern const OutputPin debugpin;
#endif

#ifdef GPIO_MOTOR
extern const OutputPin gpMotor;
#endif

#endif
