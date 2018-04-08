#ifndef HEBI_GROUP_H
#define HEBI_GROUP_H

#include "hebi_group_command.h"
#include "hebi_group_feedback.h"
#include "hebi_group_info.h"
#include <stdint.h>

#ifdef __cplusplus /* Use C linkage when compiling this C library header from C++ */
extern "C" {
#endif

/**
 * \example command_group_example.c
 * Sending commands to a group of modules.
 * \example command_persist_settings_group_example.c
 * How to indicate to a group of modules to save their current settings (so that
 * they persist through reboot).
 * \example command_settings_group_example.c
 * Sending setting commands to a group of modules.
 * \example command_spring_constants_group_example.c
 * Sending and checking spring constants for a group of modules.
 * \example command_control_strategy_group_example.c
 * Sending control strategies for a group of modules.
 * \example feedback_async_group_example.c
 * Getting feedback from a group through a callback from the API at regular intervals.
 * \example feedback_sync_group_example.c
 * Getting feedback from a group through blocking calls to the API.
 * \example info_sync_group_example.c
 * Getting info from a group through blocking calls to the API.
 */

/**
 * \brief The C-style's API representation of a group.
 *
 * Do not inherit from this; only obtain pointers through the API!
 *
 * Represents a connection to a group of modules. Sends commands to and receives
 * feedback from the group.
 *
 * @author Matthew Tesch < matt @ hebirobotics.com >
 * @since 25 Mar 2015
 */
typedef struct _HebiGroup* HebiGroupPtr;

/**
 * \brief Define a type for a group feedback handling function.
 */
typedef void (*GroupFeedbackHandlerFunction)(HebiGroupFeedbackPtr, void* user_data);

// TODO: state all failure cases for set/get functions.
// TODO: trigger 'persist' on setting name and gains guaranteed? confusing b/c of side effects? worse not to?
// TODO: logging

/**
 * \brief Returns the number of modules in a group.
 *
 * \param group The group to send this command to.
 *
 * \returns the number of modules in 'group'.
 */
int hebiGroupGetNumberOfModules(HebiGroupPtr group);

/**
 * \brief Sends a command to the given group, requesting an acknowledgement of
 * transmission to be sent back.
 *
 * \param group The group to send this command to.
 * \param command The HebiGroupCommand object containing information to be sent to
 * the group.
 * \param timeout_ms Indicates how many milliseconds to wait for a response
 * after sending a packet.  For typical networks, '15' ms is a value that can be
 * reasonably expected to encompass the time required for a round-trip
 * transmission.
 *
 * \returns '0' if an acknowledgement was successfully received (guaranteeing
 * the group received this command), or a negative number for an error otherwise.
 *
 * Note: A non-zero return does not indicate a specific failure, and may
 * result from an error while sending or simply a timeout/dropped response
 * packet after a successful transmission.
 */
int hebiGroupSendCommandWithAcknowledgement(HebiGroupPtr group,
  HebiGroupCommandPtr command,
  int timeout_ms);

/**
 * \brief Sends a command to the given group without requesting an
 * acknowledgement.
 *
 * Appropriate for high-frequency applications.
 *
 * \param group The group to send this command to.
 * \param command The HebiGroupCommand object containing information to be sent to
 * the group.
 *
 * \returns '0' if the command was successfully sent, a negative number otherwise.
 */
int hebiGroupSendCommand(HebiGroupPtr group,
  HebiGroupCommandPtr command);

/**
 * \brief Sets the command lifetime for the group, in milliseconds.
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
 * \param group Which group the command lifetime is being set for.
 * \param lifetime_ms The number of milliseconds which the command 'lives' for.
 * Setting a value less than or equal to '0' disables command lifetime. When
 * disabled, the hardware will continue to execute the last sent command.
 * Setting a value above the accepted maximum will set the lockout to the
 * maximum value.
 *
 * \returns '0' if command lifetime successfully set, or a negative integer if
 * value was outside of accepted range (higher than supported maximum).
 */
int hebiGroupSetCommandLifetime(HebiGroupPtr group, int32_t lifetime_ms);

/**
 * \brief Returns the current command lifetime, in milliseconds.
 *
 * \param group Which group is being queried.
 *
 * \returns The current command lifetime, in milliseconds. A value of '0' indicates
 * that commands remain active until the next command is received.
 */
int32_t hebiGroupGetCommandLifetime(HebiGroupPtr group);

/**
 * \brief Sets the feedback request loop frequency (in Hz).
 *
 * The group is queried for feedback in a background thread at this frequency,
 * and any added callbacks are called from this background thread.
 *
 * \param group Which group this frequency set is for.
 * \param frequency The feedback request loop frequency (in Hz). A value of '0'
 * is the default, and disables the feedback request thread.
 *
 * \returns '0' if feedback frequency successfully set, or a negative integer if value was
 * outside of accepted range (less than zero or faster than supported maximum).
 */
int hebiGroupSetFeedbackFrequencyHz(HebiGroupPtr group, float frequency);

/**
 * \brief Returns the current feedback request loop frequency (in Hz).
 *
 * \param group Which group is being queried.
 *
 * \returns The current feedback request loop frequency (in Hz).
 */
float hebiGroupGetFeedbackFrequencyHz(HebiGroupPtr group);

/**
 * \brief Add a function that is called whenever feedback is returned from the
 * group.
 *
 * \param group The group to attach this handler to.
 * \param handler A feedback handling function called whenever feedback is
 * received from the group.
 * \param user_data A pointer to user data which will be returned as the second
 * callback argument. This pointer can be NULL if desired.
 */
void hebiGroupRegisterFeedbackHandler(
  HebiGroupPtr group, GroupFeedbackHandlerFunction handler, void* user_data);

/**
 * \brief Removes all feedback handling functions from the queue to be called on
 * receipt of group feedback.
 *
 * \param group The group to which the handlers are attached.
 */
void hebiGroupClearFeedbackHandlers(
  HebiGroupPtr group);

/**
 * \brief Requests feedback from the group, and writes it to the provided Feedback
 * object.
 *
 * Warning: other data in the provided 'Feedback' object is erased!
 *
 * \param group The group to return feedback from.
 * \param feedback On success, the feedback read from the group are written
 * into this structure.
 * \param timeout_ms Indicates how many milliseconds to wait for a response
 * after sending a packet.  For typical networks, '15' ms is a value that can be
 * reasonably expected to encompass the time required for a round-trip
 * transmission.
 *
 * \returns '0' if feedback was returned, or a negative integer if not (i.e., connection
 * error or timeout waiting for response).
 */
int hebiGroupRequestFeedback(HebiGroupPtr group, HebiGroupFeedbackPtr feedback,
  int timeout_ms);

/**
 * \brief Requests info from the group, and writes it to the provided info
 * object.
 *
 * Warning: other data in the provided 'Info' object is erased!
 *
 * \param group The group to send this command to.
 * \param info On success, the info read from the group is written into this
 * structure.
 * \param timeout_ms Indicates how many milliseconds to wait for a response
 * after sending a packet.  For typical networks, '15' ms is a value that can be
 * reasonably expected to encompass the time required for a round-trip
 * transmission.
 *
 * \returns '0' if feedback was returned, or a negative integer if not (i.e., connection
 * error or timeout waiting for response).
 */
int hebiGroupRequestInfo(HebiGroupPtr group, HebiGroupInfoPtr info,
  int timeout_ms);

/**
 * \brief Starts logging data to a file in the local directory.
 *
 * WARNING: this function interface is not yet stable.
 *
 * \param group The group to log from.
 *
 * \returns '0' if successfully started a log, a negative value otherwise.
 */
int hebiGroupStartLog(HebiGroupPtr group);

/**
 * \brief Stops logging data to a file in the local directory.
 *
 * WARNING: this function interface is not yet stable.
 *
 * \param group The group that is logging.
 *
 * \returns '0' if successfully stops a log, a negative value otherwise (e.g., 
 * no log had been started)
 */
int hebiGroupStopLog(HebiGroupPtr group);

#ifdef __cplusplus /* End C linkage when compiling from C++ */
}
#endif

#endif // HEBI_GROUP_H
