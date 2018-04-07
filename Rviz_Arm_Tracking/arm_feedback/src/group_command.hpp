#ifndef GROUP_COMMAND_HPP
#define GROUP_COMMAND_HPP

#include "hebi_group_command.h"
#include "command.hpp"
#include <vector>

namespace hebi {

/**
 * \brief A list of Command objects appropriate for sending to a Group of
 * modules; the size() must match the number of modules in the group.
 */
class GroupCommand final
{
  public:
    #ifndef DOXYGEN_OMIT_INTERNAL
    /**
     * C-style group command object.
     * NOTE: this should not be used except by library functions!
     */
    HebiGroupCommandPtr const internal_;
    #endif // DOXYGEN_OMIT_INTERNAL

  private:
    /**
     * True if this object is responsible for creating and destroying the
     * underlying C pointer; false otherwise.
     */
    const bool manage_pointer_lifetime_;
    /**
     * The number of modules in this group command.
     */
    const int number_of_modules_;
    /**
     * The list of Command subobjects
     */
    std::vector<Command> commands_;

  public:
    /**
     * \brief Create a group command with the specified number of modules.
     */
    GroupCommand(int number_of_modules);

    /**
     * \brief Destructor cleans up group command object as necessary.
     */
    virtual ~GroupCommand() noexcept; /* annotating specified destructor as noexcept is best-practice */

    /**
     * \brief Returns the number of module commands in this group command.
     */
    int size() const;

    /**
     * brief Access the command for an individual module.
     */
    Command& operator[](int index);
};

} // namespace hebi

#endif // GROUP_COMMAND_HPP
