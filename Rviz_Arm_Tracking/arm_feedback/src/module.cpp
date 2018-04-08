#include "module.hpp"
#include "hebi_lookup.h" // For hebiReleaseModule

namespace hebi {

#ifndef DOXYGEN_OMIT_INTERNAL
void callbackWrapper(HebiFeedbackPtr feedback, void* user_data)
{
  ((Module*)user_data)->callAttachedHandlers(feedback);
}
#endif // DOXYGEN_OMIT_INTERNAL

void Module::callAttachedHandlers(HebiFeedbackPtr feedback)
{
  // Wrap this:
  Feedback wrapped_fbk(feedback);
  // Call handlers:
  std::lock_guard<std::mutex> lock_guard(handler_lock_);
  for (unsigned int i = 0; i < handlers_.size(); i++)
  {
    ModuleFeedbackHandler handler = handlers_[i];
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

Module::Module(HebiModulePtr module)
  : internal_(module)
{
}

Module::~Module() noexcept
{
  // Cleanup module object allocated by the C library
  if (internal_ != nullptr)
    hebiReleaseModule(internal_);
}

bool Module::sendCommand(const Command& command)
{
  return (hebiModuleSendCommand(internal_, command.internal_) == 0);
}

bool Module::sendCommandWithAcknowledgement(const Command& command, int timeout_ms)
{
  return (hebiModuleSendCommandWithAcknowledgement(internal_, command.internal_, timeout_ms) == 0);
}

bool Module::requestFeedback(Feedback* feedback, int timeout_ms)
{
  return (hebiModuleRequestFeedback(internal_, feedback->internal_, timeout_ms) == 0);
}

bool Module::requestInfo(Info* info, int timeout_ms)
{
  return (hebiModuleRequestInfo(internal_, info->internal_, timeout_ms) == 0);
}

bool Module::setFeedbackFrequencyHz(float frequency)
{
  return (hebiModuleSetFeedbackFrequencyHz(internal_, frequency) == 0);
}

float Module::getFeedbackFrequencyHz()
{
  return hebiModuleGetFeedbackFrequencyHz(internal_);
}

void Module::addFeedbackHandler(ModuleFeedbackHandler handler)
{
  std::lock_guard<std::mutex> lock_guard(handler_lock_);
  handlers_.push_back(handler);
  if (handlers_.size() == 1)
    hebiModuleRegisterFeedbackHandler(internal_, callbackWrapper, (void*)this);
}

void Module::clearFeedbackHandlers()
{
  std::lock_guard<std::mutex> lock_guard(handler_lock_);
  hebiModuleClearFeedbackHandlers(internal_);
  handlers_.clear();
}

} // namespace hebi
