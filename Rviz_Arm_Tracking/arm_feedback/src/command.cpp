#include "command.hpp"
#include <cmath>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace hebi {

Command::FloatField::FloatField(HebiCommandPtr internal, CommandFloatField field)
  : internal_(internal), field_(field)
{
}

Command::FloatField::operator bool() const
{
  return has();
}

bool Command::FloatField::has() const
{
  return (hebiCommandHasFloat(internal_, field_) == 1);
}

float Command::FloatField::get() const
{
  return hebiCommandGetFloat(internal_, field_);
}

void Command::FloatField::set(float value)
{
  hebiCommandSetFloat(internal_, field_, value);
}

void Command::FloatField::clear()
{
  hebiCommandClearFloat(internal_, field_);
}

Command::HighResAngleField::HighResAngleField(HebiCommandPtr internal, CommandHighResAngleField field)
  : internal_(internal), field_(field)
{
}

Command::HighResAngleField::operator bool() const
{
  return has();
}

bool Command::HighResAngleField::has() const
{
  return (hebiCommandHasHighResAngle(internal_, field_) == 1);
}

double Command::HighResAngleField::get() const
{
  int64_t revolutions;
  float radian_offset;
  hebiCommandGetHighResAngle(internal_, field_, &revolutions, &radian_offset);
  return ((double)revolutions * 2.0 * M_PI + (double)radian_offset);
}

void Command::HighResAngleField::get(int64_t* revolutions, float* radian_offset) const
{
  hebiCommandGetHighResAngle(internal_, field_, revolutions, radian_offset);
}

void Command::HighResAngleField::set(double radians)
{
  double revolutions_raw = radians / 2.0 / M_PI;
  double revolutions_int;
  double radian_offset = std::modf (revolutions_raw, &revolutions_int);
  radian_offset = radian_offset * 2.0 * M_PI;
  hebiCommandSetHighResAngle(internal_, field_, isnan(revolutions_int) ? 0 : (int64_t)revolutions_int, (float)radian_offset); // Preserve 'NAN' messages by just setting the floating point part.
}

void Command::HighResAngleField::set(int64_t revolutions, float radian_offset)
{
  hebiCommandSetHighResAngle(internal_, field_, revolutions, radian_offset);
}

void Command::HighResAngleField::clear()
{
  hebiCommandClearHighResAngle(internal_, field_);
}

Command::NumberedFloatField::NumberedFloatField(HebiCommandPtr internal, CommandNumberedFloatField field)
  : internal_(internal), field_(field)
{
}

bool Command::NumberedFloatField::has(int fieldNumber) const
{
  return (hebiCommandHasNumberedFloat(internal_, field_, fieldNumber) == 1);
}

float Command::NumberedFloatField::get(int fieldNumber) const
{
  return hebiCommandGetNumberedFloat(internal_, field_, fieldNumber);
}

void Command::NumberedFloatField::set(int fieldNumber, float value)
{
  hebiCommandSetNumberedFloat(internal_, field_, fieldNumber, value);
}

void Command::NumberedFloatField::clear(int fieldNumber)
{
  hebiCommandClearNumberedFloat(internal_, field_, fieldNumber);
}

Command::BoolField::BoolField(HebiCommandPtr internal, CommandBoolField field)
  : internal_(internal), field_(field)
{
}

bool Command::BoolField::has() const
{
  return (hebiCommandHasBool(internal_, field_) == 1);
}

bool Command::BoolField::get() const
{
  return (hebiCommandGetBool(internal_, field_) == 1);
}

void Command::BoolField::set(bool value)
{
  hebiCommandSetBool(internal_, field_, value ? 1 : 0);
}

void Command::BoolField::clear()
{
  hebiCommandClearBool(internal_, field_);
}

Command::StringField::StringField(HebiCommandPtr internal, CommandStringField field)
  : internal_(internal), field_(field)
{
}

Command::StringField::operator bool() const
{
  return has();
}

bool Command::StringField::has() const
{
  return (hebiCommandHasString(internal_, field_) == 1);
}

std::string Command::StringField::get() const
{
  return hebiCommandGetString(internal_, field_);
}

void Command::StringField::set(const std::string& value)
{
  hebiCommandSetString(internal_, field_, value.c_str());
}

void Command::StringField::clear()
{
  hebiCommandClearString(internal_, field_);
}

Command::FlagField::FlagField(HebiCommandPtr internal, CommandFlagField field)
  : internal_(internal), field_(field)
{
}

Command::FlagField::operator bool() const
{
  return has();
}

bool Command::FlagField::has() const
{
  return (hebiCommandHasFlag(internal_, field_) == 1);
}

void Command::FlagField::set()
{
  hebiCommandSetFlag(internal_, field_, 0);
}

void Command::FlagField::clear()
{
  hebiCommandSetFlag(internal_, field_, 1);
}

Command::LedField::LedField(HebiCommandPtr internal, CommandLedField field)
  : internal_(internal), field_(field)
{
}

bool Command::LedField::hasColor() const
{
  return (hebiCommandHasLedColor(internal_, field_) == 1);
}

bool Command::LedField::hasModuleControl() const
{
  return (hebiCommandHasLedModuleControl(internal_, field_) == 1);
}

Color Command::LedField::getColor() const
{
  uint8_t r, g, b;
  hebiCommandGetLedColor(internal_, field_, &r, &g, &b);
  return Color(r, g, b);
}

void Command::LedField::setOverrideColor(const Color& new_color)
{
  hebiCommandSetLedOverrideColor(internal_, field_,
    new_color.getRed(), new_color.getGreen(), new_color.getBlue());
}

void Command::LedField::setModuleControl()
{
  hebiCommandSetLedModuleControl(internal_, field_);
}

void Command::LedField::clear()
{
  hebiCommandClearLed(internal_, field_);
}

Command::Command()
  : internal_(hebiCommandCreate()), manage_pointer_lifetime_(true),
    settings_(internal_),
    actuator_(internal_),
    debug_(internal_, CommandNumberedFloatDebug),
    led_(internal_, CommandLedLed)
{
}

Command::Command(HebiCommandPtr command)
  : internal_(command), manage_pointer_lifetime_(false),
    settings_(internal_),
    actuator_(internal_),
    debug_(internal_, CommandNumberedFloatDebug),
    led_(internal_, CommandLedLed)
{
}

Command::~Command() noexcept
{
  if (manage_pointer_lifetime_ && internal_ != nullptr)
    hebiCommandDestroy(internal_);
}

Command::Command(Command&& other)
  : internal_(other.internal_), manage_pointer_lifetime_(other.manage_pointer_lifetime_),
    settings_(internal_),
    actuator_(internal_),
    debug_(internal_, CommandNumberedFloatDebug),
    led_(internal_, CommandLedLed)
{
  // NOTE: it would be nice to also cleanup the actual internal pointer of other
  // but alas we cannot change a const variable.  (Really, it would be great if
  // the 'manage pointer' variable was also const...)
  other.manage_pointer_lifetime_ = false;
}

// Submessage getters
Command::Settings& Command::settings()
{
  return settings_;
}

Command::Actuator& Command::actuator()
{
  return actuator_;
}

Command::NumberedFloatField& Command::debug()
{
  return debug_;
}
Command::LedField& Command::led()
{
  return led_;
}

} // namespace hebi
