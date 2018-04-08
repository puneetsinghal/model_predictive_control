#ifndef HEBI_MODULE_H
#define HEBI_MODULE_H

#include "hebi_command.h"
#include "hebi_feedback.h"
#include "hebi_info.h"
#include <stdint.h>

#ifdef __cplusplus /* Use C linkage when compiling this C library header from C++ */
extern "C" {
#endif

/**
 * \example command_module_example.c
 * Sending commands to a module.
 * \example feedback_sync_module_example.c
 * Getting feedback from a module through blocking calls to the API.
 */

/**
 * \brief The C-style's API representation of a module.
 *
 * Do not inherit from this; only obtain pointers through the API!
 *
 * Represents a connection to a single module.  Sends commands to and receives
 * feedback from the module.
 *
 * @author Matthew Tesch < matt @ hebirobotics.com >
 * @since 11 Jan 2015
 */
typedef struct _HebiModule* HebiModulePtr;

/**
 * \brief Define a type for a feedback handling function.
 */
typedef void (*FeedbackHandlerFunction)(HebiFeedbackPtr, void* user_data);

/**
 * \brief Sends a command to the given module, requesting an acknowledgement of
 * transmission to be sent back.
 *
 * \param module The module to send this command to.
 * \param command The HebiCommand object containing information to be sent to
 * the module.
 * \param timeout_ms Indicates how many milliseconds to wait for a response
 * after sending a packet.  For typical networks, '15' ms is a value that can be
 * reasonably expected to encompass the time required for a round-trip
 * transmission.
 *
 * \returns '0' if an acknowledgement was successfully received (guaranteeing
 * the module received this command), or a negative number for an error otherwise.
 *
 * Note: A non-zero return does not indicate a specific failure, and may
 * result from an error while sending or simply a timeout/dropped response
 * packet after a successful transmission.
 */
int hebiModuleSendCommandWithAcknowledgement(HebiModulePtr module,
  HebiCommandPtr command,
  int timeout_ms);

/**
 * \brief Sends a command to the given module without requesting an
 * acknowledgement.
 *
 * Appropriate for high-frequency applications.
 *
 * \param module The module to send this command to.
 * \param command The HebiCommand object containing information to be sent to
 * the module.
 *
 * \returns '0' if the command was successfully sent, a negative number otherwise.
 */
int hebiModuleSendCommand(HebiModulePtr module,
  HebiCommandPtr command);

/**
 * \brief Sets the command lifetime for the module, in milliseconds.
 *
 * The command lifetime is the duration for which a sent command remains active.
 * If the hardware does not receive further commands within the specified time
 * frame, all local controllers get deactivated. This is a safety feature to
 * mitigate the risk of accidents in case programs get interrupted in an unsafe
 * state, e.g., on program exceptions or during a network fault.
 *
 * Additionally, supporting hardware does not accept commands from any other
 * sources during the lifetime of a command. This mitigates the risk of other
 * users accidentally sending conflicting targets from, e.g., the GUI.
 *
 * \param module Which module the command lifetime is being set for.
 * \param lifetime_s The number of milliseconds which the command 'lives' for.
 * Setting a value less than or equal to '0' disables command lifetime. When
 * disabled, the hardware will continue to execute the last sent command.
 * Setting a value above the accepted maximum will set the lockout to the
 * maximum value.
 *
 * \returns '0' if command lifetime successfully set, or a negative integer if
 * value was outside of accepted range (higher than supported maximum).
 */
int hebiModuleSetCommandLifetime(HebiModulePtr module, int32_t lifetime_ms);

/**
 * \brief Returns the current command lifetime, in milliseconds.
 *
 * \param module Which module is being queried.
 *
 * \returns The current command lifetime, in milliseconds. A value of '0' indicates
 * that commands remain active until the next command is received.
 */
int32_t hebiModuleGetCommandLifetime(HebiModulePtr module);

/**
 * \brief Sets the feedback request loop frequency (in Hz).
 *
 * The module is queried for feedback in a background thread at this frequency,
 * and any added callbacks are called from this background thread.
 *
 * \param module Which module this frequency set is for.
 * \param frequency The feedback request loop frequency (in Hz). A value of '0'
 * is the default, and disables the feedback request thread.
 *
 * \returns '0' if feedback frequency successfully set, or a negative integer if value was
 * outside of accepted range (less than zero or faster than supported maximum).
 */
int hebiModuleSetFeedbackFrequencyHz(HebiModulePtr module, float frequency);

/**
 * \brief Returns the current feedback request loop frequency (in Hz).
 *
 * \param module Which module is being queried.
 *
 * \returns The current feedback request loop frequency (in Hz).
 */
float hebiModuleGetFeedbackFrequencyHz(HebiModulePtr module);

/**
 * \brief Add a function that is called whenever feedback is returned from the
 * module.
 *
 * \param module The module to attach this handler to.
 * \param handler A feedback handling function called whenever feedback is
 * received from the module.
 * \param user_data A pointer to user data which will be returned as the second
 * callback argument. This pointer can be NULL if desired.
 */
void hebiModuleRegisterFeedbackHandler(
  HebiModulePtr module, FeedbackHandlerFunction handler, void* user_data);

/**
 * \brief Removes all feedback handling functions from the queue to be called on
 * receipt of module feedback.
 *
 * \param module The module to which the handlers are attached.
 */
void hebiModuleClearFeedbackHandlers(
  HebiModulePtr module);

/**
 * \brief Requests feedback from the module, and writes it to the provided
 * Feedback object.
 *
 * Warning: other data in the provided 'Feedback' object is erased!
 *
 * \param module The module to return feedback from.
 * \param feedback On success, the feedback read from the module are written
 * into this structure.
 * \param timeout_ms Indicates how many milliseconds to wait for a response
 * after sending a packet.  For typical networks, '15' ms is a value that can be
 * reasonably expected to encompass the time required for a round-trip
 * transmission.
 *
 * \returns '0' if feedback was returned, or a negative integer if not (i.e., connection
 * error or timeout waiting for response).
 */
int hebiModuleRequestFeedback(HebiModulePtr module, HebiFeedbackPtr feedback,
  int timeout_ms);

/**
 * \brief Requests info from the module, and writes it to the provided info
 * object.
 * Warning: other data in the provided 'Info' object is erased!
 *
 * \param module The module to send this command to.
 * \param info On success, the info read from the module is written into this
 * structure.
 * \param timeout_ms Indicates how many milliseconds to wait for a response
 * after sending a packet.  For typical networks, '15' ms is a value that can be
 * reasonably expected to encompass the time required for a round-trip
 * transmission.
 *
 * \returns '0' if feedback was returned, or a negative integer if not (i.e., connection
 * error or timeout waiting for response).
 */
int hebiModuleRequestInfo(HebiModulePtr module, HebiInfoPtr info,
  int timeout_ms);

#ifdef __cplusplus /* End C linkage when compiling from C++ */
}
#endif

#endif // HEBI_MODULE_H
