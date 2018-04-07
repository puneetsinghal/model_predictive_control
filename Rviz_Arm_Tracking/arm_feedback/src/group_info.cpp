#include "group_info.hpp"

namespace hebi {

GroupInfo::GroupInfo(int number_of_modules)
 : internal_(hebiGroupInfoCreate(number_of_modules)),
   manage_pointer_lifetime_(true),
   number_of_modules_(number_of_modules)
{
  // TODO: throw exception for NULL internal ptr?
  for (int i = 0; i < number_of_modules_; i++)
    infos_.emplace_back(hebiGroupInfoGetModuleInfo(internal_, i));
}

GroupInfo::~GroupInfo() noexcept
{
  if (manage_pointer_lifetime_ && internal_ != nullptr)
    hebiGroupInfoDestroy(internal_);
}

int GroupInfo::size() const
{
  return number_of_modules_;
}

const Info& GroupInfo::operator[](int index) const
{
  return infos_[index];
}

} // namespace hebi
