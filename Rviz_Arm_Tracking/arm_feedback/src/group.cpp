#include "group.hpp"
#include "hebi_lookup.h" // For hebiReleaseGroup

namespace hebi {

#ifndef DOXYGEN_OMIT_INTERNAL
void callbackWrapper(HebiGroupFeedbackPtr group_feedback, void* user_data)
{
  ((Group*)user_data)->callAttachedHandlers(group_feedback);
}
#endif // DOXYGEN_OMIT_INTERNAL

void Group::callAttachedHandlers(HebiGroupFeedbackPtr group_feedback)
{
  // Wrap this:
  GroupFeedback wrapped_fbk(group_feedback);
  // Call handlers:
  std::lock_guard<std::mutex> lock_guard(handler_lock_);
  for (unsigned int i = 0; i < handlers_.size(); i++)
  {
    GroupFeedbackHandler handler = handlers_[i];
    // TODO: be sure to catch exceptions!
    try
    {
      handler(&wrapped_fbk);
    }
    catch (...)
    {
      // TODO: print error or something?
    }
  }
}

Group::Group(HebiGroupPtr group)
  : internal_(group), number_of_modules_(hebiGroupGetNumberOfModules(internal_))
{
}

Group::~Group() noexcept
{
  // Cleanup group object allocated by the C library
  if (internal_ != nullptr)
    hebiReleaseGroup(internal_);
}

int Group::size()
{
  return number_of_modules_;
}

bool Group::sendCommand(const GroupCommand& group_command)
{
  return (hebiGroupSendCommand(internal_, group_command.internal_) == 0);
}

bool Group::sendCommandWithAcknowledgement(const GroupCommand& group_command, int timeout_ms)
{
  return (hebiGroupSendCommandWithAcknowledgement(internal_, group_command.internal_, timeout_ms) == 0);
}
    
bool Group::requestFeedback(GroupFeedback* feedback, int timeout_ms)
{
  return (hebiGroupRequestFeedback(internal_, feedback->internal_, timeout_ms) == 0);
}

bool Group::requestInfo(GroupInfo* info, int timeout_ms)
{
  return (hebiGroupRequestInfo(internal_, info->internal_, timeout_ms) == 0);
}

bool Group::startLog()
{
  return (hebiGroupStartLog(internal_) == 0);
}

bool Group::stopLog()
{
  return (hebiGroupStopLog(internal_) == 0);
}

bool Group::setFeedbackFrequencyHz(float frequency)
{
  return (hebiGroupSetFeedbackFrequencyHz(internal_, frequency) == 0);
}

float Group::getFeedbackFrequencyHz()
{
  return hebiGroupGetFeedbackFrequencyHz(internal_);
}

void Group::addFeedbackHandler(GroupFeedbackHandler handler)
{
  std::lock_guard<std::mutex> lock_guard(handler_lock_);
  handlers_.push_back(handler);
  if (handlers_.size() == 1) // (i.e., this was the first one)
    hebiGroupRegisterFeedbackHandler(internal_, callbackWrapper, (void*)this);
}

void Group::clearFeedbackHandlers()
{
  std::lock_guard<std::mutex> lock_guard(handler_lock_);
  hebiGroupClearFeedbackHandlers(internal_);
  handlers_.clear();
}

} // namespace hebi
