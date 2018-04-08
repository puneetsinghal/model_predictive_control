/* This file has been automatically generated. Do not edit by hand. */

#ifndef HEBI_COMMAND_H
#define HEBI_COMMAND_H

#include "stdint.h"

#ifdef __cplusplus /* Use C linkage when compiling this C library header from C++ */
extern "C" {
#endif

typedef struct _HebiCommand* HebiCommandPtr;

HebiCommandPtr hebiCommandCreate();

// Define all fields
typedef enum CommandFloatField {
CommandFloatVelocity, ///Velocity of the module output (post-spring), in radians/second.
CommandFloatTorque, ///Torque at the module output, in N * m.
CommandFloatPositionKp, ///Proportional PID gain for position
CommandFloatPositionKi, ///Integral PID gain for position
CommandFloatPositionKd, ///Derivative PID gain for position
CommandFloatPositionFeedForward, ///Feed forward term for position (this term is multiplied by the target and added to the output).
CommandFloatPositionDeadZone, ///Error values within +/- this value from zero are treated as zero (in terms of computed proportional output, input to numerical derivative, and accumulated integral error).
CommandFloatPositionIClamp, ///Maximum allowed value for the output of the integral component of the PID loop; the integrated error is not allowed to exceed value that will generate this number.
CommandFloatPositionPunch, ///Constant offset to the position PID output outside of the deadzone; it is added when the error is positive and subtracted when it is negative.
CommandFloatPositionMinTarget, ///Minimum allowed value for input to the PID controller
CommandFloatPositionMaxTarget, ///Maximum allowed value for input to the PID controller
CommandFloatPositionTargetLowpass, ///A simple lowpass filter applied to the target set point; needs to be between 0 and 1.  At each timestep: x_t = x_t * a + x_{t-1} * (1 - a).
CommandFloatPositionMinOutput, ///Output from the PID controller is limited to a minimum of this value.
CommandFloatPositionMaxOutput, ///Output from the PID controller is limited to a maximum of this value.
CommandFloatPositionOutputLowpass, ///A simple lowpass filter applied to the controller output; needs to be between 0 and 1.  At each timestep: x_t = x_t * a + x_{t-1} * (1 - a).
CommandFloatVelocityKp, ///Proportional PID gain for velocity
CommandFloatVelocityKi, ///Integral PID gain for velocity
CommandFloatVelocityKd, ///Derivative PID gain for velocity
CommandFloatVelocityFeedForward, ///Feed forward term for velocity (this term is multiplied by the target and added to the output).
CommandFloatVelocityDeadZone, ///Error values within +/- this value from zero are treated as zero (in terms of computed proportional output, input to numerical derivative, and accumulated integral error).
CommandFloatVelocityIClamp, ///Maximum allowed value for the output of the integral component of the PID loop; the integrated error is not allowed to exceed value that will generate this number.
CommandFloatVelocityPunch, ///Constant offset to the velocity PID output outside of the deadzone; it is added when the error is positive and subtracted when it is negative.
CommandFloatVelocityMinTarget, ///Minimum allowed value for input to the PID controller
CommandFloatVelocityMaxTarget, ///Maximum allowed value for input to the PID controller
CommandFloatVelocityTargetLowpass, ///A simple lowpass filter applied to the target set point; needs to be between 0 and 1.  At each timestep: x_t = x_t * a + x_{t-1} * (1 - a).
CommandFloatVelocityMinOutput, ///Output from the PID controller is limited to a minimum of this value.
CommandFloatVelocityMaxOutput, ///Output from the PID controller is limited to a maximum of this value.
CommandFloatVelocityOutputLowpass, ///A simple lowpass filter applied to the controller output; needs to be between 0 and 1.  At each timestep: x_t = x_t * a + x_{t-1} * (1 - a).
CommandFloatTorqueKp, ///Proportional PID gain for torque
CommandFloatTorqueKi, ///Integral PID gain for torque
CommandFloatTorqueKd, ///Derivative PID gain for torque
CommandFloatTorqueFeedForward, ///Feed forward term for torque (this term is multiplied by the target and added to the output).
CommandFloatTorqueDeadZone, ///Error values within +/- this value from zero are treated as zero (in terms of computed proportional output, input to numerical derivative, and accumulated integral error).
CommandFloatTorqueIClamp, ///Maximum allowed value for the output of the integral component of the PID loop; the integrated error is not allowed to exceed value that will generate this number.
CommandFloatTorquePunch, ///Constant offset to the torque PID output outside of the deadzone; it is added when the error is positive and subtracted when it is negative.
CommandFloatTorqueMinTarget, ///Minimum allowed value for input to the PID controller
CommandFloatTorqueMaxTarget, ///Maximum allowed value for input to the PID controller
CommandFloatTorqueTargetLowpass, ///A simple lowpass filter applied to the target set point; needs to be between 0 and 1.  At each timestep: x_t = x_t * a + x_{t-1} * (1 - a).
CommandFloatTorqueMinOutput, ///Output from the PID controller is limited to a minimum of this value.
CommandFloatTorqueMaxOutput, ///Output from the PID controller is limited to a maximum of this value.
CommandFloatTorqueOutputLowpass, ///A simple lowpass filter applied to the controller output; needs to be between 0 and 1.  At each timestep: x_t = x_t * a + x_{t-1} * (1 - a).
CommandFloatSpringConstant, ///The spring constant of the module.
} CommandFloatField;

typedef enum CommandHighResAngleField {
CommandHighResAnglePosition, ///Position of the module output (post-spring), in radians.
} CommandHighResAngleField;

typedef enum CommandNumberedFloatField {
CommandNumberedFloatDebug, ///Values for internal debug functions (channel 1-9 available).
} CommandNumberedFloatField;

typedef enum CommandBoolField {
CommandBoolPositionDOnError, ///Controls whether the Kd term uses the "derivative of error" or "derivative of measurement."  When the setpoints have step inputs or are noisy, setting this to @c false can eliminate corresponding spikes or noise in the output.
CommandBoolVelocityDOnError, ///Controls whether the Kd term uses the "derivative of error" or "derivative of measurement."  When the setpoints have step inputs or are noisy, setting this to @c false can eliminate corresponding spikes or noise in the output.
CommandBoolTorqueDOnError, ///Controls whether the Kd term uses the "derivative of error" or "derivative of measurement."  When the setpoints have step inputs or are noisy, setting this to @c false can eliminate corresponding spikes or noise in the output.
} CommandBoolField;

typedef enum CommandStringField {
CommandStringName, ///Sets the name for this module. Name must be null-terminated character string for the name; must be <= 20 characters.
CommandStringFamily, ///Sets the family for this module. Name must be null-terminated character string for the family; must be <= 20 characters.
} CommandStringField;

typedef enum CommandFlagField {
CommandFlagSaveCurrentSettings, ///Indicates if the module should save the current values of all of its settings.
} CommandFlagField;

typedef enum CommandEnumField {
CommandEnumControlStrategy, ///How the position, velocity, and torque PID loops are connected in order to control motor PWM.
} CommandEnumField;

typedef enum CommandLedField {
CommandLedLed, ///The module's LED.
} CommandLedField;

/**
 * Returns value for given field. If HasFloat() returns 0, this results in
 * undefined behavior.
 */
float hebiCommandGetFloat(HebiCommandPtr, CommandFloatField);
/**
 * Indicates whether the specified value is set; returns '1' for yes and '0' for
 * no.
 */
int hebiCommandHasFloat(HebiCommandPtr, CommandFloatField);
/**
 * Sets the given field.
 */
void hebiCommandSetFloat(HebiCommandPtr, CommandFloatField, float);
/**
 * Clears the given field, so that no value is sent.
 */
void hebiCommandClearFloat(HebiCommandPtr, CommandFloatField);

/**
 * Returns value for given field. If HasHighResAngle() returns 0, this results in
 * undefined behavior.
 */
void hebiCommandGetHighResAngle(HebiCommandPtr, CommandHighResAngleField, int64_t* int_part, float* dec_part);
/**
 * Indicates whether the specified value is set; returns '1' for yes and '0' for
 * no.
 */
int hebiCommandHasHighResAngle(HebiCommandPtr, CommandHighResAngleField);
/**
 * Sets the given field.
 */
void hebiCommandSetHighResAngle(HebiCommandPtr, CommandHighResAngleField, int64_t int_part, float dec_part);
/**
 * Clears the given field, so that no value is sent.
 */
void hebiCommandClearHighResAngle(HebiCommandPtr, CommandHighResAngleField);

/**
 * Returns value for given field. If HasNumberedFloat() for this number returns
 * 0, this results in undefined behavior.
 */
float hebiCommandGetNumberedFloat(HebiCommandPtr, CommandNumberedFloatField, int);
/**
 * Indicates whether the specified value is set; returns '1' for yes and '0' for
 * no.
 */
int hebiCommandHasNumberedFloat(HebiCommandPtr, CommandNumberedFloatField, int);
/**
 * Sets indexed value.
 */
void hebiCommandSetNumberedFloat(HebiCommandPtr, CommandNumberedFloatField, int, float);
/**
 * Clears indexed value, so that no value is sent for this number.
 */
void hebiCommandClearNumberedFloat(HebiCommandPtr, CommandNumberedFloatField, int);

/**
 * Returns value of given field, '1' for true or '0' for false. If HasBool()
 * returns 0, this results in undefined behavior.
 */
int hebiCommandGetBool(HebiCommandPtr, CommandBoolField);
/**
 * Indicates whether the specified value is set; returns '1' for yes and '0' for
 * no.
 */
int hebiCommandHasBool(HebiCommandPtr, CommandBoolField);
/**
 * Sets the given field; use '1' for true and '0' for false.
 */
void hebiCommandSetBool(HebiCommandPtr, CommandBoolField, int);
/**
 * Clears the given field, so that no value is sent.
 */
void hebiCommandClearBool(HebiCommandPtr, CommandBoolField);

/**
 * Returns value for given field. If HasString() returns 0, this results in
 * undefined behavior.
 */
const char* hebiCommandGetString(HebiCommandPtr, CommandStringField);
/**
 * Indicates whether the specified value is set; returns '1' for yes and '0' for
 * no.
 */
int hebiCommandHasString(HebiCommandPtr, CommandStringField);
/**
 * Sets the given field.
 */
void hebiCommandSetString(HebiCommandPtr, CommandStringField, const char *);
/**
 * Clears the given field, so that no value is sent.
 */
void hebiCommandClearString(HebiCommandPtr, CommandStringField);

/**
 * Checks whether this flag is set.  Returns '1' for yes, '0' for no.
 */
int hebiCommandHasFlag(HebiCommandPtr, CommandFlagField);
/**
 * Sets or clears a flag value. A value of '1' sets this flag and a value of '0'
 * clears this flag.
 */
void hebiCommandSetFlag(HebiCommandPtr, CommandFlagField, int);

/**
 * Returns value for given field. If HasEnum() returns 0, this results in
 * undefined behavior.
 */
int hebiCommandGetEnum(HebiCommandPtr, CommandEnumField);
/**
 * Indicates whether the specified value is set; returns '1' for yes and '0' for
 * no.
 */
int hebiCommandHasEnum(HebiCommandPtr, CommandEnumField);
/**
 * Sets the given field.
 */
void hebiCommandSetEnum(HebiCommandPtr, CommandEnumField, int);
/**
 * Clears the given field, so that no value is sent.
 */
void hebiCommandClearEnum(HebiCommandPtr, CommandEnumField);

/**
 * Returns led color into the three output integers (each 0-255). If HasLedColor() returns 0, this results in
 * undefined behavior.
 */
void hebiCommandGetLedColor(HebiCommandPtr, CommandLedField, uint8_t *r, uint8_t *g, uint8_t *b);
/**
 * Indicates whether the led color is set; returns '1' for yes and '0' for no. For command-style messages, this refers
 * to a command to override the module's default control of the LED.
 */
int hebiCommandHasLedColor(HebiCommandPtr, CommandLedField);
/**
 * Returns '1' if this message indicates that the module should resume control of the LED.  A '0' can indicate either
 * an override command (if and only if HasLedColor() returns '1'), or no information about the LED (i.e., the module
 * should maintain it's current state regarding the LED).
 */
int hebiCommandHasLedModuleControl(HebiCommandPtr, CommandLedField);
/**
 * Commands a color that overrides the module's control of the LED.
 */
void hebiCommandSetLedOverrideColor(HebiCommandPtr, CommandLedField, uint8_t r, uint8_t g, uint8_t b);
/**
 * Sets the module to regain control of the LED.
 */
void hebiCommandSetLedModuleControl(HebiCommandPtr, CommandLedField);
/**
 * Clears the given LED field, so that the module maintains its previous state of LED control/color (i.e., does not have an override color command or an explicit 'module control' command).
 */
void hebiCommandClearLed(HebiCommandPtr, CommandLedField);

void hebiCommandDestroy(HebiCommandPtr);

#ifdef __cplusplus /* End C linkage when compiling from C++ */
} // extern "C"
#endif

#endif // HEBI_COMMAND_H
