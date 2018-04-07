/* This file has been automatically generated. Do not edit by hand. */

#ifndef HEBI_INFO_H
#define HEBI_INFO_H

#include "stdint.h"

#ifdef __cplusplus /* Use C linkage when compiling this C library header from C++ */
extern "C" {
#endif

typedef struct _HebiInfo* HebiInfoPtr;

HebiInfoPtr hebiInfoCreate();

// Define all fields
typedef enum InfoFloatField {
InfoFloatPositionKp, ///Proportional PID gain for position
InfoFloatPositionKi, ///Integral PID gain for position
InfoFloatPositionKd, ///Derivative PID gain for position
InfoFloatPositionFeedForward, ///Feed forward term for position (this term is multiplied by the target and added to the output).
InfoFloatPositionDeadZone, ///Error values within +/- this value from zero are treated as zero (in terms of computed proportional output, input to numerical derivative, and accumulated integral error).
InfoFloatPositionIClamp, ///Maximum allowed value for the output of the integral component of the PID loop; the integrated error is not allowed to exceed value that will generate this number.
InfoFloatPositionPunch, ///Constant offset to the position PID output outside of the deadzone; it is added when the error is positive and subtracted when it is negative.
InfoFloatPositionMinTarget, ///Minimum allowed value for input to the PID controller
InfoFloatPositionMaxTarget, ///Maximum allowed value for input to the PID controller
InfoFloatPositionTargetLowpass, ///A simple lowpass filter applied to the target set point; needs to be between 0 and 1.  At each timestep: x_t = x_t * a + x_{t-1} * (1 - a).
InfoFloatPositionMinOutput, ///Output from the PID controller is limited to a minimum of this value.
InfoFloatPositionMaxOutput, ///Output from the PID controller is limited to a maximum of this value.
InfoFloatPositionOutputLowpass, ///A simple lowpass filter applied to the controller output; needs to be between 0 and 1.  At each timestep: x_t = x_t * a + x_{t-1} * (1 - a).
InfoFloatVelocityKp, ///Proportional PID gain for velocity
InfoFloatVelocityKi, ///Integral PID gain for velocity
InfoFloatVelocityKd, ///Derivative PID gain for velocity
InfoFloatVelocityFeedForward, ///Feed forward term for velocity (this term is multiplied by the target and added to the output).
InfoFloatVelocityDeadZone, ///Error values within +/- this value from zero are treated as zero (in terms of computed proportional output, input to numerical derivative, and accumulated integral error).
InfoFloatVelocityIClamp, ///Maximum allowed value for the output of the integral component of the PID loop; the integrated error is not allowed to exceed value that will generate this number.
InfoFloatVelocityPunch, ///Constant offset to the velocity PID output outside of the deadzone; it is added when the error is positive and subtracted when it is negative.
InfoFloatVelocityMinTarget, ///Minimum allowed value for input to the PID controller
InfoFloatVelocityMaxTarget, ///Maximum allowed value for input to the PID controller
InfoFloatVelocityTargetLowpass, ///A simple lowpass filter applied to the target set point; needs to be between 0 and 1.  At each timestep: x_t = x_t * a + x_{t-1} * (1 - a).
InfoFloatVelocityMinOutput, ///Output from the PID controller is limited to a minimum of this value.
InfoFloatVelocityMaxOutput, ///Output from the PID controller is limited to a maximum of this value.
InfoFloatVelocityOutputLowpass, ///A simple lowpass filter applied to the controller output; needs to be between 0 and 1.  At each timestep: x_t = x_t * a + x_{t-1} * (1 - a).
InfoFloatTorqueKp, ///Proportional PID gain for torque
InfoFloatTorqueKi, ///Integral PID gain for torque
InfoFloatTorqueKd, ///Derivative PID gain for torque
InfoFloatTorqueFeedForward, ///Feed forward term for torque (this term is multiplied by the target and added to the output).
InfoFloatTorqueDeadZone, ///Error values within +/- this value from zero are treated as zero (in terms of computed proportional output, input to numerical derivative, and accumulated integral error).
InfoFloatTorqueIClamp, ///Maximum allowed value for the output of the integral component of the PID loop; the integrated error is not allowed to exceed value that will generate this number.
InfoFloatTorquePunch, ///Constant offset to the torque PID output outside of the deadzone; it is added when the error is positive and subtracted when it is negative.
InfoFloatTorqueMinTarget, ///Minimum allowed value for input to the PID controller
InfoFloatTorqueMaxTarget, ///Maximum allowed value for input to the PID controller
InfoFloatTorqueTargetLowpass, ///A simple lowpass filter applied to the target set point; needs to be between 0 and 1.  At each timestep: x_t = x_t * a + x_{t-1} * (1 - a).
InfoFloatTorqueMinOutput, ///Output from the PID controller is limited to a minimum of this value.
InfoFloatTorqueMaxOutput, ///Output from the PID controller is limited to a maximum of this value.
InfoFloatTorqueOutputLowpass, ///A simple lowpass filter applied to the controller output; needs to be between 0 and 1.  At each timestep: x_t = x_t * a + x_{t-1} * (1 - a).
InfoFloatSpringConstant, ///The spring constant of the module.
} InfoFloatField;

typedef enum InfoBoolField {
InfoBoolPositionDOnError, ///Controls whether the Kd term uses the "derivative of error" or "derivative of measurement."  When the setpoints have step inputs or are noisy, setting this to @c false can eliminate corresponding spikes or noise in the output.
InfoBoolVelocityDOnError, ///Controls whether the Kd term uses the "derivative of error" or "derivative of measurement."  When the setpoints have step inputs or are noisy, setting this to @c false can eliminate corresponding spikes or noise in the output.
InfoBoolTorqueDOnError, ///Controls whether the Kd term uses the "derivative of error" or "derivative of measurement."  When the setpoints have step inputs or are noisy, setting this to @c false can eliminate corresponding spikes or noise in the output.
} InfoBoolField;

typedef enum InfoStringField {
InfoStringName, ///Sets the name for this module. Name must be null-terminated character string for the name; must be <= 20 characters.
InfoStringFamily, ///Sets the family for this module. Name must be null-terminated character string for the family; must be <= 20 characters.
} InfoStringField;

typedef enum InfoFlagField {
InfoFlagSaveCurrentSettings, ///Indicates if the module should save the current values of all of its settings.
} InfoFlagField;

typedef enum InfoEnumField {
InfoEnumControlStrategy, ///How the position, velocity, and torque PID loops are connected in order to control motor PWM.
} InfoEnumField;

typedef enum InfoLedField {
InfoLedLed, ///The module's LED.
} InfoLedField;

/**
 * Returns value for given field. If HasFloat() returns 0, this results in
 * undefined behavior.
 */
float hebiInfoGetFloat(HebiInfoPtr, InfoFloatField);
/**
 * Indicates whether the specified value is set; returns '1' for yes and '0' for
 * no.
 */
int hebiInfoHasFloat(HebiInfoPtr, InfoFloatField);

/**
 * Returns value of given field, '1' for true or '0' for false. If HasBool()
 * returns 0, this results in undefined behavior.
 */
int hebiInfoGetBool(HebiInfoPtr, InfoBoolField);
/**
 * Indicates whether the specified value is set; returns '1' for yes and '0' for
 * no.
 */
int hebiInfoHasBool(HebiInfoPtr, InfoBoolField);

/**
 * Returns value for given field. If HasString() returns 0, this results in
 * undefined behavior.
 */
const char* hebiInfoGetString(HebiInfoPtr, InfoStringField);
/**
 * Indicates whether the specified value is set; returns '1' for yes and '0' for
 * no.
 */
int hebiInfoHasString(HebiInfoPtr, InfoStringField);

/**
 * Checks whether this flag is set.  Returns '1' for yes, '0' for no.
 */
int hebiInfoHasFlag(HebiInfoPtr, InfoFlagField);

/**
 * Returns value for given field. If HasEnum() returns 0, this results in
 * undefined behavior.
 */
int hebiInfoGetEnum(HebiInfoPtr, InfoEnumField);
/**
 * Indicates whether the specified value is set; returns '1' for yes and '0' for
 * no.
 */
int hebiInfoHasEnum(HebiInfoPtr, InfoEnumField);

/**
 * Returns led color into the three output integers (each 0-255). If HasLedColor() returns 0, this results in
 * undefined behavior.
 */
void hebiInfoGetLedColor(HebiInfoPtr, InfoLedField, uint8_t *r, uint8_t *g, uint8_t *b);
/**
 * Indicates whether the led color is set; returns '1' for yes and '0' for no. For command-style messages, this refers
 * to a command to override the module's default control of the LED.
 */
int hebiInfoHasLedColor(HebiInfoPtr, InfoLedField);

void hebiInfoDestroy(HebiInfoPtr);

#ifdef __cplusplus /* End C linkage when compiling from C++ */
} // extern "C"
#endif

#endif // HEBI_INFO_H
