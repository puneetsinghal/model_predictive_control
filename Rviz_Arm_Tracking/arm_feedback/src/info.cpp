#include "info.hpp"

namespace hebi {

Info::FloatField::FloatField(HebiInfoPtr internal, InfoFloatField field)
  : internal_(internal), field_(field)
{
}

Info::FloatField::operator bool() const
{
  return has();
}

bool Info::FloatField::has() const
{
  return (hebiInfoHasFloat(internal_, field_) == 1);
}

float Info::FloatField::get() const
{
  return hebiInfoGetFloat(internal_, field_);
}

Info::BoolField::BoolField(HebiInfoPtr internal, InfoBoolField field)
  : internal_(internal), field_(field)
{
}

bool Info::BoolField::has() const
{
  return (hebiInfoHasBool(internal_, field_) == 1);
}

bool Info::BoolField::get() const
{
  return (hebiInfoGetBool(internal_, field_) == 1);
}

Info::StringField::StringField(HebiInfoPtr internal, InfoStringField field)
  : internal_(internal), field_(field)
{
}

Info::StringField::operator bool() const
{
  return has();
}

bool Info::StringField::has() const
{
  return (hebiInfoHasString(internal_, field_) == 1);
}

std::string Info::StringField::get() const
{
  return hebiInfoGetString(internal_, field_);
}

Info::FlagField::FlagField(HebiInfoPtr internal, InfoFlagField field)
  : internal_(internal), field_(field)
{
}

Info::FlagField::operator bool() const
{
  return has();
}

bool Info::FlagField::has() const
{
  return (hebiInfoHasFlag(internal_, field_) == 1);
}

Info::LedField::LedField(HebiInfoPtr internal, InfoLedField field)
  : internal_(internal), field_(field)
{
}

bool Info::LedField::hasColor() const
{
  return (hebiInfoHasLedColor(internal_, field_) == 1);
}

Color Info::LedField::getColor() const
{
  uint8_t r, g, b;
  hebiInfoGetLedColor(internal_, field_, &r, &g, &b);
  return Color(r, g, b);
}

Info::Info()
  : internal_(hebiInfoCreate()), manage_pointer_lifetime_(true),
    settings_(internal_),
    led_(internal_, InfoLedLed)
{
}

Info::Info(HebiInfoPtr info)
  : internal_(info), manage_pointer_lifetime_(false),
    settings_(internal_),
    led_(internal_, InfoLedLed)
{
}

Info::~Info() noexcept
{
  if (manage_pointer_lifetime_ && internal_ != nullptr)
    hebiInfoDestroy(internal_);
}

Info::Info(Info&& other)
  : internal_(other.internal_), manage_pointer_lifetime_(other.manage_pointer_lifetime_),
    settings_(internal_),
    led_(internal_, InfoLedLed)
{
  // NOTE: it would be nice to also cleanup the actual internal pointer of other
  // but alas we cannot change a const variable.  (Really, it would be great if
  // the 'manage pointer' variable was also const...)
  other.manage_pointer_lifetime_ = false;
}

// Submessage getters
const Info::Settings& Info::settings() const
{
  return settings_;
}

const Info::LedField& Info::led() const
{
  return led_;
}

} // namespace hebi
