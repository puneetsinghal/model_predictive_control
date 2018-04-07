#ifndef HEBI_LOOKUP_H
#define HEBI_LOOKUP_H

#include "hebi_module.h"
#include "hebi_group.h"
#include "hebi_mac_address.h"

#ifdef __cplusplus /* Use C linkage when compiling this C library header from C++ */
extern "C" {
#endif

/**
 * \example lookup_group_example.c
 * How to lookup a group - simple.
 * \example lookup_group_general_example.c
 * How to lookup a group - generalized using command line arguments.
 * \example lookup_module_example.c
 * How to lookup a module - simple.
 * \example lookup_module_general_example.c
 * How to lookup a module - generalized using command line arguments.
 * \example lookup_helpers.c
 * Two primary functions -- getModuleFromArgs and getGroupFromArgs -- to
 * assist with lookups from command line arguments.
 */

/**
 * Maintains a registry of network-connected modules and returns Module and
 * Group objects to the user. Only one Lookup object is needed per application.
 *
 * @author Matthew Tesch < matt @ hebirobotics.com >
 * @since 29 Oct 2014
 */
typedef struct _HebiLookup *HebiLookupPtr;

/**
 * \brief Create a Lookup instance.
 * 
 * Lookup created by this function must be released with 'hebiDeleteLookup'
 * when no longer needed.
 *
 * Note that this call invokes a background thread to query the network for
 * modules at regular intervals.
 */
HebiLookupPtr hebiCreateLookup();
/**
 * \brief Frees resources created by the lookup object.
 *
 * Lookup object should no longer be used after this function is called!
 * Note that background query thread is stopped by this function.
 */
void hebiDeleteLookup(HebiLookupPtr lookup);
/**
 * \brief Frees all resources created by the library.  Note: any calls to the
 * HEBI library functions after this will result in undefined behavior!
 */
void hebiCleanup();
/**
 * \brief Get a module with the given mac address.
 *
 * Blocking call which waits to get a pointer to a Module object with the
 * given parameters.  Times out after timeout_msec milliseconds. Must be
 * released when use is complete via the hebiReleaseModule function.
 * 
 * @param lookup A valid HebiLookup object.
 * @param address Physical mac address of the given module (serves as unique id).
 * @param timeout_ms Timeout in milliseconds.  A value of -1 blocks until
 * a module is found, and a value of 0 returns immediately if no module with
 * that address is currently known by the Lookup class.
 * @returns NULL if no module found in allotted time; pointer to newly allocated
 * module object corresponding to the given parameters otherwise.
 */
HebiModulePtr hebiGetModuleFromMac(HebiLookupPtr lookup, const HebiMacAddress* address, long timeout_ms);
/**
 * \brief Get a module with the given name and family.
 *
 * Blocking call which waits to get a pointer to a Module object with the
 * given parameters.  Times out after timeout_msec milliseconds. Must be
 * released when use is complete via the hebiReleaseGroup function.
 * 
 * @param lookup A valid HebiLookup object.
 * @param name The given name of the module, as viewable in the HEBI GUI.
 * Must be a null-terminated string.
 * @param family The given family of the module, as viewable in the HEBI GUI.
 * Must be a null-terminated string.
 * @param timeout_ms Timeout in milliseconds.  A value of -1 blocks until
 * a module is found, and a value of 0 returns immediately if no module with
 * that address is currently known by the Lookup class.
 * @returns NULL if no module found in allotted time; pointer to newly allocated
 * module object corresponding to the given parameters otherwise.
 */
HebiModulePtr hebiGetModuleFromName(HebiLookupPtr lookup, const char* name, const char* family, long timeout_ms);
/**
 * \brief Get a group of modules with the given MAC addresses.
 *
 * If any given modules are not found, no group is created.
 *
 * Blocking call which waits to get a pointer to a Group object with the
 * given parameters.  Times out after timeout_msec milliseconds. Must be
 * released when use is complete via the hebiReleaseGroup function.
 * 
 * @param lookup A valid HebiLookup object.
 * @param addresses An array of pointers to physical mac addresses of the given
 * modules. Length of the array must equal num_addresses.
 * @param num_addresses Length of the addresses array of pointers (number of
 * pointers in the array, not cumulative size of objects they point to).
 * @param timeout_ms Timeout in milliseconds.  A value of -1 blocks until
 * a module is found, and a value of 0 returns immediately if no module with
 * that address is currently known by the Lookup class.
 * @returns NULL if matching group not found in allotted time; pointer to newly
 * allocated group object otherwise.
 */
HebiGroupPtr hebiGetGroupFromMacs(HebiLookupPtr lookup, const HebiMacAddress* addresses, int num_addresses, long timeout_ms);
/**
 * \brief Get a group with modules matching the given names and families.
 *
 * If only one family is given, it is used for all modules.  Otherwise, number of
 * names and families must match. If any given modules are not found, no group is
 * created.
 *
 * Blocking call which waits to get a pointer to a Group object with the
 * given parameters.  Times out after timeout_msec milliseconds. Must be
 * released when use is complete via the hebiReleaseGroup function.
 * 
 * @param lookup A valid HebiLookup object.
 * @param names The given names of the modules, as viewable in the HEBI GUI. Must
 * be a list of pointers to null-terminated strings. The number of pointers must
 * match the num_names parameter.
 * @param num_names The number of pointers to null-terminated strings given
 * by the names parameter.
 * @param families The given families of the modules, as viewable in the HEBI
 * GUI. Must be a list of pointers to null-terminated strings. The number of
 * pointers must match the num_families parameter. Note that a single string
 * (with corresponding value of num_families == 1) will be used with each name in
 * the names list.
 * @param num_families The number of pointers to null-terminated strings given
 * by the families parameter. Note that this must either be 1, or be equal to
 * num_names.
 * @param timeout_ms Timeout in milliseconds.  A value of -1 blocks until
 * a module is found, and a value of 0 returns immediately if no module with
 * that address is currently known by the Lookup class.
 * @returns NULL if matching group not found in allotted time; pointer to newly
 * allocated group object otherwise.
 */
HebiGroupPtr hebiGetGroupFromNames(HebiLookupPtr lookup, const char* const * names, int num_names, const char* const * families, int num_families, long timeout_ms);
/**
 * \brief Get a group with all modules known to the lookup with the given family.
 *
 * Group contains all modules with the given family, regardless of name.
 *
 * Blocking call which waits to get a pointer to a Group object with the
 * given parameters.  Times out after timeout_msec milliseconds. Must be
 * released when use is complete via the hebiReleaseGroup function.
 * 
 * @param lookup A valid HebiLookup object.
 * @param family The given family of the modules, as viewable in the HEBI GUI.
 * Must be a null-terminated string.
 * @param timeout_ms Timeout in milliseconds.  A value of -1 blocks until
 * a module is found, and a value of 0 returns immediately if no module with
 * that address is currently known by the Lookup class.
 * @returns NULL if matching group not found in allotted time; pointer to newly
 * allocated group object otherwise.
 */
HebiGroupPtr hebiGetGroupFromFamily(HebiLookupPtr lookup, const char* family, long timeout_ms);
/**
 * \brief Get a group with all modules connected to module with the given MAC
 * address.
 *
 * Modules in group will be ordered depth-first, starting with the most proximal
 * module.
 *
 * Blocking call which waits to get a pointer to a Group object with the
 * given parameters.  Times out after timeout_msec milliseconds. Must be
 * released when use is complete via the hebiReleaseGroup function.
 * 
 * @param lookup A valid HebiLookup object.
 * @param address Physical mac address of the given module (serves as unique id).
 * @param timeout_ms Timeout in milliseconds.  A value of -1 blocks until
 * a module is found, and a value of 0 returns immediately if no module with
 * that address is currently known by the Lookup class.
 * @returns NULL if matching group not found in allotted time; pointer to newly
 * allocated group object otherwise.
 */
HebiGroupPtr hebiGetConnectedGroupFromMac(HebiLookupPtr lookup, const HebiMacAddress* address, long timeout_ms);
/**
 * \brief Get a group with all modules connected to module with the given name
 * and family
 *
 * Modules in group will be ordered depth-first, starting with the most proximal
 * module.
 *
 * Blocking call which waits to get a pointer to a Group object with the
 * given parameters.  Times out after timeout_msec milliseconds. Must be
 * released when use is complete via the hebiReleaseGroup function.
 * 
 * @param lookup A valid HebiLookup object.
 * @param name The given name of the key module, as viewable in the HEBI GUI.
 * Must be a null-terminated string.
 * @param family The given family of the key module, as viewable in the HEBI GUI.
 * Must be a null-terminated string.
 * @param timeout_ms Timeout in milliseconds.  A value of -1 blocks until
 * a module is found, and a value of 0 returns immediately if no module with
 * that address is currently known by the Lookup class.
 * @returns NULL if matching group not found in allotted time; pointer to newly
 * allocated group object otherwise.
 */
HebiGroupPtr hebiGetConnectedGroupFromName(HebiLookupPtr lookup, const char* name, const char* family, long timeout_ms);
/**
 * \brief Display the contents of the module registry -- i.e., which modules have
 * been found by the lookup.
 *
 * @param lookup A valid HebiLookup object.
 */
void hebiPrintLookupTable(HebiLookupPtr lookup);
/**
 * \brief Release resources for a given module; module should not be used after
 * this call.
 *
 * @param module A valid HebiModule object.
 */
void hebiReleaseModule(HebiModulePtr module);
/**
 * \brief Release resources for a given group; group should not be used after
 * this call.
 *
 * @param group A valid HebiGroup object.
 */
void hebiReleaseGroup(HebiGroupPtr group);

#ifdef __cplusplus /* End C linkage when compiling from C++ */
}
#endif

#endif // HEBI_LOOKUP_H
