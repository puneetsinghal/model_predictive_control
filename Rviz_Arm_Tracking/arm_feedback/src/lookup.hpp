#ifndef LOOKUP_HPP
#define LOOKUP_HPP

#include "hebi_lookup.h"
#include "mac_address.hpp"
#include "module.hpp"
#include "group.hpp"

#include <memory> // For unique_ptr
#include <vector>

namespace hebi {

/**
 * \example lookup_group_example.cpp
 * How to lookup a group - simple.
 * \example lookup_group_general_example.cpp
 * How to lookup a group - generalized using command line arguments.
 * \example lookup_module_example.cpp
 * How to lookup a module - simple.
 * \example lookup_module_general_example.cpp
 * How to lookup a module - generalized using command line arguments.
 * \example lookup_helpers.cpp
 * File with parsing of command line arguments and actual group generation used
 * by general examples above.
 */

/**
 * \brief Maintains a registry of network-connected modules and returns Module and
 * Group objects to the user.
 *
 * Only one Lookup object is needed per application.
 *
 * @author Matthew Tesch < matt @ hebirobotics.com >
 * @since 18 Feb 2016
 */
class Lookup final
{
  private:
    static const long DEFAULT_TIMEOUT = 500;

    /**
     * \internal C-style lookup object
     */
    HebiLookupPtr lookup_;

  public:
    /**
     * \brief Creates a Lookup object which can create Module and Group
     * references.
     * Typically, only one Lookup object should exist at a time.
     *
     * Note that this call invokes a background thread to query the network for
     * modules at regular intervals.
     */
    Lookup();

    /**
     * \brief Destructor frees all resources created by Lookup object, and stops the
     * background query thread.
     */
    virtual ~Lookup() noexcept; /* annotating specified destructor as noexcept is best-practice */

    /**
     * \brief Displays the current contents of the module registry on stdout -- i.e.,
     * which modules have been found by the lookup.
     *
     * \unstable{This function is not yet part of the stable API, and is subject
     * to change in future versions.}
     */
    void printTable();

    /**
     * \brief Get a module with the given name and family.
     *
     * Blocking call which returns a reference to a Module object with the given
     * parameters. Times out after timeout_msec milliseconds.
     *
     * @param name The given name of the module, as viewable in the HEBI GUI.
     * @param family The given family of the module, as viewable in the HEBI GUI.
     * @param timeout_ms Timeout in milliseconds.  A value of -1 blocks until
     * a module is found, and a value of 0 returns immediately if no module with
     * that address is currently known by the Lookup class.
     * @returns A unique_ptr with no reference if no module found in allotted
     * time, or reference to a newly allocated module object corresponding to
     * the given parameters otherwise.
     */
    std::unique_ptr<Module> getModuleFromName(const std::string& name, const std::string& family, long timeout_ms=DEFAULT_TIMEOUT);
    
    /**
     * \brief Get a module with the given mac address.
     *
     * Blocking call which returns a reference to a Module object with the given
     * parameters. Times out after timeout_msec milliseconds.
     * @param address Physical mac address of the given module (serves as unique id).
     * @param timeout_ms Timeout in milliseconds.  A value of -1 blocks until
     * a module is found, and a value of 0 returns immediately if no module with
     * that address is currently known by the Lookup class.
     * @returns A unique_ptr with no reference if no module found in allotted
     * time, or reference to a newly allocated module object corresponding to
     * the given parameters otherwise.
     */
    std::unique_ptr<Module> getModuleFromMac(const MacAddress& address, long timeout_ms=DEFAULT_TIMEOUT);

    /**
     * \brief Get a group from modules with the given names and families.
     *
     * If one of the input vectors is of length one, then that element is
     * assumed to pair with each item in the other input vector.
     *
     * Blocking call which returns a reference to a Group object with the given
     * parameters. Times out after timeout_msec milliseconds.
     *
     * @param names A list of names of desired group modules, as viewable in the
     * HEBI GUI.  If of length one, this name is paired with each given family
     * @param families A list of families of desired group modules, as viewable
     * in the HEBI GUI.  If of length one, this family is paried with each given
     * name.
     * @param timeout_ms Timeout in milliseconds.  A value of -1 blocks until
     * a group is found, and a value of 0 returns immediately if no group with
     * that address is currently known by the Lookup class.
     * @returns A unique_ptr with no reference if no module found in allotted
     * time, or reference to a newly allocated module object corresponding to
     * the given parameters otherwise.
     */
    std::unique_ptr<Group> getGroupFromNames(const std::vector<std::string>& names, const std::vector<std::string>& families, long timeout_ms=DEFAULT_TIMEOUT);

    /**
     * \brief Get a group from modules with the given mac addresses.
     *
     * Blocking call which returns a reference to a Group object with the given
     * parameters. Times out after timeout_msec milliseconds.
     *
     * @param addresses List of physical mac addresses for desired group modules.
     * @param timeout_ms Timeout in milliseconds.  A value of -1 blocks until
     * a group is found, and a value of 0 returns immediately if no group with
     * that address is currently known by the Lookup class.
     * @returns A unique_ptr with no reference if no group found in allotted
     * time, or reference to a newly allocated group object corresponding to
     * the given parameters otherwise.
     */
    std::unique_ptr<Group> getGroupFromMacs(const std::vector<MacAddress>& addresses, long timeout_ms=DEFAULT_TIMEOUT);

    /**
     * \brief Get a group from all known modules with the given family.
     *
     * Blocking call which returns a reference to a Group object with the given
     * parameters. Times out after timeout_msec milliseconds.
     *
     * @param family The family of each of the desired group modules.
     * @param timeout_ms Timeout in milliseconds.  A value of -1 blocks until
     * a group is found, and a value of 0 returns immediately if no group with
     * that address is currently known by the Lookup class.
     * @returns A unique_ptr with no reference if no group found in allotted
     * time, or reference to a newly allocated group object corresponding to
     * the given parameters otherwise.
     */
    std::unique_ptr<Group> getGroupFromFamily(const std::string& family, long timeout_ms=DEFAULT_TIMEOUT);

    /**
     * \brief Get a group from all modules known to connect to a module with the
     * given name and family.
     *
     * Blocking call which returns a reference to a Group object with the given
     * parameters. Times out after timeout_msec milliseconds.
     *
     * @param name The given name of the module, as viewable in the HEBI GUI, to
     * form the group from.
     * @param family The given family of the module, as viewable in the HEBI
     * GUI, to form the group from.
     * @param timeout_ms Timeout in milliseconds.  A value of -1 blocks until
     * a group is found, and a value of 0 returns immediately if no group with
     * that address is currently known by the Lookup class.
     * @returns A unique_ptr with no reference if no group found in allotted
     * time, or reference to a newly allocated group object corresponding to
     * the given parameters otherwise.
     */
    std::unique_ptr<Group> getConnectedGroupFromName(const std::string& name, const std::string& family, long timeout_ms=DEFAULT_TIMEOUT);

    /**
     * \brief Get a group from all modules known to connect to a module with the
     * given mac address.
     *
     * Blocking call which returns a reference to a Group object with the given
     * parameters. Times out after timeout_msec milliseconds.
     *
     * @param address Physical mac address of the module to form the group from.
     * @param timeout_ms Timeout in milliseconds.  A value of -1 blocks until
     * a group is found, and a value of 0 returns immediately if no group with
     * that address is currently known by the Lookup class.
     * @returns A unique_ptr with no reference if no group found in allotted
     * time, or reference to a newly allocated group object corresponding to
     * the given parameters otherwise.
     */
    std::unique_ptr<Group> getConnectedGroupFromMac(const MacAddress& address, long timeout_ms=DEFAULT_TIMEOUT);
   
  private:
    /**
     * Disable copy and move constructors and assignment operators
     */
    HEBI_DISABLE_COPY_MOVE(Lookup)
};

} // namespace hebi

#endif // LOOKUP_HPP
