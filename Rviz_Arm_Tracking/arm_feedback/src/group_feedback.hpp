#ifndef GROUP_FEEDBACK_HPP
#define GROUP_FEEDBACK_HPP

#include "hebi_group_feedback.h"
#include "feedback.hpp"
#include <vector>

namespace hebi {

/**
 * \brief A list of Feedback objects that can be received from a Group of
 * modules; the size() must match the number of modules in the group.
 */
class GroupFeedback final
{
  public:
    #ifndef DOXYGEN_OMIT_INTERNAL
    /**
     * C-style group feedback object.
     * NOTE: this should not be used except by library functions!
     */
    HebiGroupFeedbackPtr const internal_;
    #endif // DOXYGEN_OMIT_INTERNAL

  private:
    /**
     * True if this object is responsible for creating and destroying the
     * underlying C pointer; false otherwise.
     */
    const bool manage_pointer_lifetime_;
    /**
     * The number of modules in this group feedback.
     */
    const int number_of_modules_;
    /**
     * The list of Feedback subobjects
     */
    std::vector<Feedback> feedbacks_;

  public:
    /**
     * \brief Create a group feedback with the specified number of modules.
     */
    GroupFeedback(int number_of_modules);
    #ifndef DOXYGEN_OMIT_INTERNAL
    /**
     * Wraps an existing C-style feedback object; object lifetime is assumed to
     * be managed by the caller.
     * NOTE: this should not be used except by internal library functions!
     */
    GroupFeedback(HebiGroupFeedbackPtr group_feedback);
    #endif // DOXYGEN_OMIT_INTERNAL

    /**
     * \brief Destructor cleans up group feedback object as necessary.
     */
    virtual ~GroupFeedback() noexcept; /* annotating specified destructor as noexcept is best-practice */

    /**
     * \brief Returns the number of module feedbacks in this group feedback.
     */
    int size() const;

    /**
     * \brief Access the feedback for an individual module.
     */
    const Feedback& operator[](int index) const;
};

} // namespace hebi

#endif // GROUP_FEEDBACK_HPP
