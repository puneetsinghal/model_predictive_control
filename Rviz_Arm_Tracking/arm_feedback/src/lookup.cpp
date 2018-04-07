#include "lookup.hpp"
#include <algorithm> // For std::transform
#include <iterator> // For std::back_inserter on Windows.

namespace hebi {

Lookup::Lookup()
{
  lookup_ = hebiCreateLookup();
}

Lookup::~Lookup() noexcept
{
  hebiDeleteLookup(lookup_);
}

void Lookup::printTable()
{
  hebiPrintLookupTable(lookup_);
}

std::unique_ptr<Module> Lookup::getModuleFromName(const std::string& name, const std::string& family_name, long timeout_ms)
{
  std::unique_ptr<Module> ptr;
  HebiModulePtr module = hebiGetModuleFromName(lookup_, name.c_str(), family_name.c_str(), timeout_ms);
  if (module != nullptr)
    ptr.reset(new Module(module));
  return ptr;
}

std::unique_ptr<Module> Lookup::getModuleFromMac(const MacAddress& address, long timeout_ms)
{
  std::unique_ptr<Module> ptr;
  HebiModulePtr module = hebiGetModuleFromMac(lookup_, &(address.internal_), timeout_ms);
  if (module != nullptr)
    ptr.reset(new Module(module));
  return ptr;
}

std::unique_ptr<Group> Lookup::getGroupFromNames(const std::vector<std::string>& names, const std::vector<std::string>& families, long timeout_ms)
{
  std::unique_ptr<Group> ptr;
  std::vector<const char *> names_cstrs;
  std::vector<const char *> families_cstrs;
  names_cstrs.reserve(names.size());
  families_cstrs.reserve(families.size());
  std::transform(std::begin(names), std::end(names),
    std::back_inserter(names_cstrs), std::mem_fn(&std::string::c_str));
  std::transform(std::begin(families), std::end(families),
    std::back_inserter(families_cstrs), std::mem_fn(&std::string::c_str));
  HebiGroupPtr group = hebiGetGroupFromNames(lookup_, names_cstrs.data(), names_cstrs.size(), families_cstrs.data(), families_cstrs.size(), timeout_ms);
  if (group != nullptr)
    ptr.reset(new Group(group));
  return ptr;
}

std::unique_ptr<Group> Lookup::getGroupFromMacs(const std::vector<MacAddress>& addresses, long timeout_ms)
{
  std::unique_ptr<Group> ptr;
  std::vector<HebiMacAddress> addresses_c;
  addresses_c.reserve(addresses.size());
  std::transform(std::begin(addresses), std::end(addresses),
    std::back_inserter(addresses_c), [] (const MacAddress& addr) { return addr.internal_; });
  HebiGroupPtr group = hebiGetGroupFromMacs(lookup_, addresses_c.data(), addresses.size(), timeout_ms);
  if (group != nullptr)
    ptr.reset(new Group(group));
  return ptr;
}

std::unique_ptr<Group> Lookup::getGroupFromFamily(const std::string& family, long timeout_ms)
{
  std::unique_ptr<Group> ptr;
  HebiGroupPtr group = hebiGetGroupFromFamily(lookup_, family.c_str(), timeout_ms);
  if (group != nullptr)
    ptr.reset(new Group(group));
  return ptr;
}

std::unique_ptr<Group> Lookup::getConnectedGroupFromName(const std::string& name, const std::string& family_name, long timeout_ms)
{
  std::unique_ptr<Group> ptr;
  HebiGroupPtr group = hebiGetConnectedGroupFromName(lookup_, name.c_str(), family_name.c_str(), timeout_ms);
  if (group != nullptr)
    ptr.reset(new Group(group));
  return ptr;
}

std::unique_ptr<Group> Lookup::getConnectedGroupFromMac(const MacAddress& address, long timeout_ms)
{
  std::unique_ptr<Group> ptr;
  HebiGroupPtr group = hebiGetConnectedGroupFromMac(lookup_, &(address.internal_), timeout_ms);
  if (group != nullptr)
    ptr.reset(new Group(group));
  return ptr;
}

} // namespace hebi
