#include "group_command.hpp"

namespace hebi {

GroupCommand::GroupCommand(int number_of_modules)
 : internal_(hebiGroupCommandCreate(number_of_modules)),
   manage_pointer_lifetime_(true),
   number_of_modules_(number_of_modules)
{
  // TODO: throw exception for NULL internal ptr?
  for (int i = 0; i < number_of_modules_; i++)
    commands_.emplace_back(hebiGroupCommandGetModuleCommand(internal_, i));
}

GroupCommand::~GroupCommand() noexcept
{
  if (manage_pointer_lifetime_ && internal_ != nullptr)
    hebiGroupCommandDestroy(internal_);
}

int GroupCommand::size() const
{
  return number_of_modules_;
}

Command& GroupCommand::operator[](int index)
{
  return commands_[index];
}

} // namespace hebi
