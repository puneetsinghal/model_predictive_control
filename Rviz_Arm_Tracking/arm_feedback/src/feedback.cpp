#include "feedback.hpp"
#include <cmath>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace hebi {

Feedback::FloatField::FloatField(HebiFeedbackPtr internal, FeedbackFloatField field)
  : internal_(internal), field_(field)
{
}

Feedback::FloatField::operator bool() const
{
  return has();
}

bool Feedback::FloatField::has() const
{
  return (hebiFeedbackHasFloat(internal_, field_) == 1);
}

float Feedback::FloatField::get() const
{
  return hebiFeedbackGetFloat(internal_, field_);
}

Feedback::HighResAngleField::HighResAngleField(HebiFeedbackPtr internal, FeedbackHighResAngleField field)
  : internal_(internal), field_(field)
{
}

Feedback::HighResAngleField::operator bool() const
{
  return has();
}

bool Feedback::HighResAngleField::has() const
{
  return (hebiFeedbackHasHighResAngle(internal_, field_) == 1);
}

double Feedback::HighResAngleField::get() const
{
  int64_t revolutions;
  float radian_offset;
  hebiFeedbackGetHighResAngle(internal_, field_, &revolutions, &radian_offset);
  return ((double)revolutions * 2.0 * M_PI + (double)radian_offset);
}

void Feedback::HighResAngleField::get(int64_t* revolutions, float* radian_offset) const
{
  hebiFeedbackGetHighResAngle(internal_, field_, revolutions, radian_offset);
}

Feedback::NumberedFloatField::NumberedFloatField(HebiFeedbackPtr internal, FeedbackNumberedFloatField field)
  : internal_(internal), field_(field)
{
}

bool Feedback::NumberedFloatField::has(int fieldNumber) const
{
  return (hebiFeedbackHasNumberedFloat(internal_, field_, fieldNumber) == 1);
}

float Feedback::NumberedFloatField::get(int fieldNumber) const
{
  return hebiFeedbackGetNumberedFloat(internal_, field_, fieldNumber);
}

Feedback::Vector3fField::Vector3fField(HebiFeedbackPtr internal, FeedbackVector3fField field)
  : internal_(internal), field_(field)
{
}

Feedback::Vector3fField::operator bool() const
{
  return has();
}

bool Feedback::Vector3fField::has() const
{
  return (hebiFeedbackHasVector3f(internal_, field_) == 1);
}

Vector3f Feedback::Vector3fField::get() const
{
  return hebiFeedbackGetVector3f(internal_, field_);
}

Feedback::LedField::LedField(HebiFeedbackPtr internal, FeedbackLedField field)
  : internal_(internal), field_(field)
{
}

bool Feedback::LedField::hasColor() const
{
  return (hebiFeedbackHasLedColor(internal_, field_) == 1);
}

Color Feedback::LedField::getColor() const
{
  uint8_t r, g, b;
  hebiFeedbackGetLedColor(internal_, field_, &r, &g, &b);
  return Color(r, g, b);
}

Feedback::Feedback()
  : internal_(hebiFeedbackCreate()), manage_pointer_lifetime_(true),
    actuator_(internal_),
    imu_(internal_),
    board_temperature_(internal_, FeedbackFloatBoardTemperature),
    processor_temperature_(internal_, FeedbackFloatProcessorTemperature),
    voltage_(internal_, FeedbackFloatVoltage),
    debug_(internal_, FeedbackNumberedFloatDebug),
    led_(internal_, FeedbackLedLed)
{
}

Feedback::Feedback(HebiFeedbackPtr feedback)
  : internal_(feedback), manage_pointer_lifetime_(false),
    actuator_(internal_),
    imu_(internal_),
    board_temperature_(internal_, FeedbackFloatBoardTemperature),
    processor_temperature_(internal_, FeedbackFloatProcessorTemperature),
    voltage_(internal_, FeedbackFloatVoltage),
    debug_(internal_, FeedbackNumberedFloatDebug),
    led_(internal_, FeedbackLedLed)
{
}

Feedback::~Feedback() noexcept
{
  if (manage_pointer_lifetime_ && internal_ != nullptr)
    hebiFeedbackDestroy(internal_);
}

Feedback::Feedback(Feedback&& other)
  : internal_(other.internal_), manage_pointer_lifetime_(other.manage_pointer_lifetime_),
    actuator_(internal_),
    imu_(internal_),
    board_temperature_(internal_, FeedbackFloatBoardTemperature),
    processor_temperature_(internal_, FeedbackFloatProcessorTemperature),
    voltage_(internal_, FeedbackFloatVoltage),
    debug_(internal_, FeedbackNumberedFloatDebug),
    led_(internal_, FeedbackLedLed)
{
  // NOTE: it would be nice to also cleanup the actual internal pointer of other
  // but alas we cannot change a const variable.  (Really, it would be great if
  // the 'manage pointer' variable was also const...)
  other.manage_pointer_lifetime_ = false;
}

// Submessage getters
const Feedback::Actuator& Feedback::actuator() const
{
  return actuator_;
}

const Feedback::Imu& Feedback::imu() const
{
  return imu_;
}

const Feedback::FloatField& Feedback::boardTemperature() const
{
  return board_temperature_;
}
const Feedback::FloatField& Feedback::processorTemperature() const
{
  return processor_temperature_;
}
const Feedback::FloatField& Feedback::voltage() const
{
  return voltage_;
}
const Feedback::NumberedFloatField& Feedback::debug() const
{
  return debug_;
}
const Feedback::LedField& Feedback::led() const
{
  return led_;
}

} // namespace hebi
