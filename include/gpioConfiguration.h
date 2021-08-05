#pragma once
/* This header file stores all of the GPIO pin output and input definitions. */

/******* Pinout Menu for wiring up robot *******
 *                                             *
 * Here for easy reference                     *
 * crane_rot_axis.attach(25);       PURPLE     *
 * crane_fa_axis.attach(26);        BROWN      *
 * crane_ca_axis.attach(27);	    YELLOW     *
 * crane_ca_2_axis.attach(32);      BLUE       *
 * crane_gripper_servo.attach(33);  WHITE      *
 * #define INTERRUPT_PIN 39                    *
 * I2C SCL: 22                                 *
 * I2C SDA: 21                                 *
************************************************/

#define headLightRelayPin 23
#define sysPwrReaderPin 34 // this needs to be an analogue pin on ADC1 not ADC2!!!

/* GPIO outputs for l298n H bridge motor driver and PWM */
#define leftMotorPosPin 16 // motor 1 pin 1  (WHITE ENA)
#define leftMotorNegPin 17 // motor 1 pin 2  (BLACK ENA)
#define rightMotorPosPin 18 // motor 2 pin 1  (GRAY ENB)
#define rightMotorNegPin 19 // motor 2 pin 2  (WHITE ENB)
#define leftMotorPwmPin 13 // motor 1 pwm pin  (BROWN ENA)
#define rightMotorPwmPin 14 // motor 2 pwm pin (PURPLE ENB)

/* Other GPIO */
#define enableWifiPortalPin 35 // AKA RTC_GPIO5 used to custom mqtt server enable

/* GPIO Output for crane servos */
#define craneYawServoPin 25
#define cranePitchServo1Pin 27
#define cranePitchServo2Pin 32
#define craneElbowServoPin 26
#define craneGripperServoPin 33