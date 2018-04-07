/**
 * Helper functions to print out module/group feedback
 */

#include <iostream>
#include <cmath>

#include "feedback.hpp"
#include "group_feedback.hpp"

void print_module_feedback(const hebi::Feedback& feedback)
{
  // The feedback object contains many fields and subobjects, depending on
  // the type of feedback. Here we get a subcommand of this object which
  // contains relevant feedback for actuators, such as position, velocity,
  // and torque:
  const auto& a_fbk = feedback.actuator();
  std::cout << "Position " << (a_fbk.position() ? a_fbk.position().get() : NAN) << " | "
            << "Velocity " << (a_fbk.velocity() ? a_fbk.velocity().get() : NAN) << " | "
            << "Torque "   << (a_fbk.torque()   ? a_fbk.torque().get()   : NAN) << std::endl;
}

void print_group_feedback(const hebi::GroupFeedback& group_feedback)
{
  for (int mod_idx = 0; mod_idx < group_feedback.size(); mod_idx++)
  {
    std::cout << "Module index: " << mod_idx << std::endl;
    print_module_feedback(group_feedback[mod_idx]);
  }
}
