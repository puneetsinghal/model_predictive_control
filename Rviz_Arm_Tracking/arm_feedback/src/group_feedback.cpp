#include "group_feedback.hpp"

namespace hebi {

GroupFeedback::GroupFeedback(int number_of_modules)
 : internal_(hebiGroupFeedbackCreate(number_of_modules)),
   manage_pointer_lifetime_(true),
   number_of_modules_(number_of_modules)
{
  // TODO: throw exception for NULL internal ptr?
  for (int i = 0; i < number_of_modules_; i++)
    feedbacks_.emplace_back(hebiGroupFeedbackGetModuleFeedback(internal_, i));
}

GroupFeedback::GroupFeedback(HebiGroupFeedbackPtr const group_feedback)
 : internal_(group_feedback),
   manage_pointer_lifetime_(false),
   number_of_modules_(hebiGroupFeedbackGetNumModules(group_feedback))
{
  // TODO: exception on invalid group_feedback?
  for (int i = 0; i < number_of_modules_; i++)
    feedbacks_.emplace_back(hebiGroupFeedbackGetModuleFeedback(internal_, i));
}

GroupFeedback::~GroupFeedback() noexcept
{
  if (manage_pointer_lifetime_ && internal_ != nullptr)
    hebiGroupFeedbackDestroy(internal_);
}

int GroupFeedback::size() const
{
  return number_of_modules_;
}

const Feedback& GroupFeedback::operator[](int index) const
{
  return feedbacks_[index];
}

} // namespace hebi
