#ifndef COMMAND_HPP
#define COMMAND_HPP

#include "hebi_command.h"
#include "color.hpp"
#include <string>
#include "util.hpp"

namespace hebi {

/// \brief Command objects have various fields that can be set; when sent to the
/// module, these fields control internal properties and setpoints.
/// 
/// This object has a hierarchical structure -- there are some direct general-purpose
/// fields at the top level, and many more specific fields contained in different
/// nested subobjects.
/// 
/// The subobjects contain references to the parent command object, and so should not
/// be used after the parent object has been destroyed.
/// 
/// The fields in the command object are typed; generally, these are optional-style
/// read/write fields (i.e., have the concept of get/set/has/clear), although the
/// return types and exact interface vary slightly between fields. Where appropriate,
/// the explicit bool operator has been overridden so that you can shortcut
/// @c if(field.has()) by calling @c if(field).
/// 
/// Although this header file can be used to look at the hierarchy of the messages,
/// in general the online documentation at apidocs.hebi.us presents this information.
/// in a more readable form.
class Command final
{
  public:
    enum ControlStrategy {
      /// The motor is not given power (equivalent to a 0 PWM value)
      Off,
      /// A direct PWM value (-1 to 1) can be sent to the motor (subject to onboard safety limiting).
      DirectPWM,
      /// A combination of the position, velocity, and torque loops with P and V feeding to T; documented on docs.hebi.us under "Control Modes"
      Strategy2,
      /// A combination of the position, velocity, and torque loops with P, V, and T feeding to PWM; documented on docs.hebi.us under "Control Modes"
      Strategy3,
      /// A combination of the position, velocity, and torque loops with P feeding to T and V feeding to PWM; documented on docs.hebi.us under "Control Modes"
      Strategy4,
    };

  // Note: this is 'protected' instead of 'private' for easier use with Doxygen
  protected:
    /// \brief A message field representable by a single-precision floating point value.
    class FloatField final
    {
      public:
        #ifndef DOXYGEN_OMIT_INTERNAL
        FloatField(HebiCommandPtr internal, CommandFloatField field);
        #endif // DOXYGEN_OMIT_INTERNAL
        /// \brief Allows casting to a bool to check if the field has a value
        /// without directly calling @c has().
        ///
        /// This can be used as in the following (assuming 'parent' is a parent message,
        /// and this field is called 'myField')
        /// \code{.cpp}
        /// Command::FloatField& f = parent.myField();
        /// if (f)
        ///   std::cout << "Field has value: " << f.get() << std::endl;
        /// else
        ///   std::cout << "Field has no value!" << std::endl;
        /// \endcode
        explicit operator bool() const;
        /// \brief True if (and only if) the field has a value.
        bool has() const;
        /// \brief If the field has a value, returns that value; otherwise,
        /// returns a default.
        float get() const;
        /// \brief Sets the field to a given value.
        void set(float value);
        /// \brief Removes any currently set value for this field.
        void clear();

      private:
        HebiCommandPtr const internal_;
        CommandFloatField const field_;

        HEBI_DISABLE_COPY_MOVE(FloatField)
    };
    /// \brief A message field for an angle measurement which does not lose
    /// precision at very high angles.
    ///
    /// This field is represented as an int64_t for the number of revolutions
    /// and a float for the radian offset from that number of revolutions.
    class HighResAngleField final
    {
      public:
        #ifndef DOXYGEN_OMIT_INTERNAL
        HighResAngleField(HebiCommandPtr internal, CommandHighResAngleField field);
        #endif // DOXYGEN_OMIT_INTERNAL
        /// \brief Allows casting to a bool to check if the field has a value
        /// without directly calling @c has().
        ///
        /// This can be used as in the following (assuming 'parent' is a parent message,
        /// and this field is called 'myField')
        /// \code{.cpp}
        /// Command::HighResAngleField& f = parent.myField();
        /// if (f)
        ///   std::cout << "Field has value: " << f.get() << std::endl;
        /// else
        ///   std::cout << "Field has no value!" << std::endl;
        /// \endcode
        explicit operator bool() const;
        /// \brief True if (and only if) the field has a value.
        bool has() const;
        /// \brief If the field has a value, returns that value as a double;
        /// otherwise, returns a default.
        ///
        /// Note that some precision might be lost converting to a double at
        /// very high number of revolutions.
        double get() const;
        /// \brief If the field has a value, returns that value in the int64
        /// and float parameters passed in; otherwise, returns a default.
        ///
        /// Note that this maintains the full precision of the underlying angle
        /// measurement, even for very large numbers of revolutions.
        ///
        /// \param revolutions The number of full revolutions
        ///
        /// \param radian_offset The offset from the given number of full
        /// revolutions.  Note that this is usually between 0 and @c 2*M_PI, but
        /// callers should not assume this.
        void get(int64_t* revolutions, float* radian_offset) const;
        /// \brief Sets the field to a given double value (in radians).  Note
        /// that double precision floating point numbers cannot represent the
        /// same angular resolution at very high magnitudes as they can at lower
        /// magnitudes.
        void set(double radians);
        /// \brief Sets the field to a given integer number of revolutions and
        /// a floating point offset from this number of revolutions.  The offset
        /// does not specifically need to fall within a certain range (i.e., can
        /// add more than a single revolution to the integer value), but should
        /// be kept relatively small (e.g., below 10,000) to avoid potential
        /// loss of precision.
        void set(int64_t revolutions, float radian_offset);
        /// \brief Removes any currently set value for this field.
        void clear();

      private:
        HebiCommandPtr const internal_;
        CommandHighResAngleField const field_;

        HEBI_DISABLE_COPY_MOVE(HighResAngleField)
    };

    /// \brief A message field containing a numbered set of single-precision
    /// floating point values.
    class NumberedFloatField final
    {
      public:
        #ifndef DOXYGEN_OMIT_INTERNAL
        NumberedFloatField(HebiCommandPtr internal, CommandNumberedFloatField field);
        #endif // DOXYGEN_OMIT_INTERNAL
        /// \brief True if (and only if) the particular numbered subvalue of
        /// this field has a value.
        ///
        /// \param fieldNumber Which subvalue to check; valid values for
        /// fieldNumber depend on the field type.
        bool has(int fieldNumber) const;
        /// \brief If the particular numbered subvalue of this field has a
        /// value, returns that value; otherwise returns a default.
        ///
        /// \param fieldNumber Which subvalue to get; valid values for
        /// fieldNumber depend on the field type.
        float get(int fieldNumber) const;
        /// \brief Sets the particular numbered subvalue of this field to a
        /// given value.
        ///
        /// \param fieldNumber Which subvalue to set; valid values for
        /// fieldNumber depend on the field type.
        void set(int fieldNumber, float value);
        /// \brief Removes any currently set value for the numbered subvalue of
        /// this field.
        ///
        /// \param fieldNumber Which subvalue to clear; valid values for
        /// fieldNumber depend on the field type.
        void clear(int fieldNumber);

      private:
        HebiCommandPtr const internal_;
        CommandNumberedFloatField const field_;

        HEBI_DISABLE_COPY_MOVE(NumberedFloatField)
    };

    /// \brief A message field representable by a bool value.
    class BoolField final
    {
      public:
        #ifndef DOXYGEN_OMIT_INTERNAL
        BoolField(HebiCommandPtr internal, CommandBoolField field);
        #endif // DOXYGEN_OMIT_INTERNAL
        /// \brief True if (and only if) the field has a value.
        bool has() const;
        /// \brief If the field has a value, returns that value; otherwise,
        /// returns false.
        bool get() const;
        /// \brief Sets the field to a given value.
        void set(bool value);
        /// \brief Removes any currently set value for this field.
        void clear();

      private:
        HebiCommandPtr const internal_;
        CommandBoolField const field_;

        HEBI_DISABLE_COPY_MOVE(BoolField)
    };

    /// \brief A message field representable by a std::string.
    class StringField final
    {
      public:
        #ifndef DOXYGEN_OMIT_INTERNAL
        StringField(HebiCommandPtr internal, CommandStringField field);
        #endif // DOXYGEN_OMIT_INTERNAL
        /// \brief Allows casting to a bool to check if the field has a value
        /// without directly calling @c has().
        ///
        /// This can be used as in the following (assuming 'parent' is a parent message,
        /// and this field is called 'myField')
        /// \code{.cpp}
        /// Command::StringField& f = parent.myField();
        /// if (f)
        ///   std::cout << "Field has value: " << f.get() << std::endl;
        /// else
        ///   std::cout << "Field has no value!" << std::endl;
        /// \endcode
        explicit operator bool() const;
        /// \brief True if (and only if) the field has a value.
        bool has() const;
        /// \brief If the field has a value, returns a copy of that value;
        /// otherwise, returns a default.
        std::string get() const;
        /// \brief Sets the field to a given value.
        void set(const std::string& value);
        /// \brief Removes any currently set value for this field.
        void clear();

      private:
        HebiCommandPtr const internal_;
        CommandStringField const field_;

        HEBI_DISABLE_COPY_MOVE(StringField)
    };

    /// \brief A two-state message field (either set/true or cleared/false).
    class FlagField final
    {
      public:
        #ifndef DOXYGEN_OMIT_INTERNAL
        FlagField(HebiCommandPtr internal, CommandFlagField field);
        #endif // DOXYGEN_OMIT_INTERNAL
        /// \brief Allows casting to a bool to check if the flag is set without
        /// directly calling @c has().
        ///
        /// This can be used as in the following (assuming 'parent' is a parent message,
        /// and this field is called 'myField')
        /// \code{.cpp}
        /// Command::FlagField& f = parent.myField();
        /// if (f)
        ///   std::cout << "Field has value: " << f.get() << std::endl;
        /// else
        ///   std::cout << "Field has no value!" << std::endl;
        /// \endcode
        explicit operator bool() const;
        /// \brief Returns @c true if the flag is set, false if it is cleared.
        bool has() const;
        /// \brief Sets this flag.
        void set();
        /// \brief Clears this flag (e.g., sets it to false/off).
        void clear();

      private:
        HebiCommandPtr const internal_;
        CommandFlagField const field_;

        HEBI_DISABLE_COPY_MOVE(FlagField)
    };

    /// \brief A message field representable by an enum of a given type.
    template <class T>
    class EnumField final
    {
      public:
        #ifndef DOXYGEN_OMIT_INTERNAL
        EnumField(HebiCommandPtr internal, CommandEnumField field)
          : internal_(internal), field_(field) {}
        #endif // DOXYGEN_OMIT_INTERNAL
        /// \brief Allows casting to a bool to check if the field has a value
        /// without directly calling @c has().
        ///
        /// This can be used as in the following (assuming 'parent' is a parent message,
        /// and this field is called 'myField')
        /// \code{.cpp}
        /// Command::EnumField& f = parent.myField();
        /// if (f)
        ///   std::cout << "Field has value: " << f.get() << std::endl;
        /// else
        ///   std::cout << "Field has no value!" << std::endl;
        /// \endcode
        explicit operator bool() const { return has(); }
        /// \brief True if (and only if) the field has a value.
        bool has() const { return (hebiCommandHasEnum(internal_, field_) == 1); }
        /// \brief If the field has a value, returns that value; otherwise,
        /// returns a default.
        T get() const { return (T)hebiCommandGetEnum(internal_, field_); }
        /// \brief Sets the field to a given value.
        void set(T value) { hebiCommandSetEnum(internal_, field_, value); }
        /// \brief Removes any currently set value for this field.
        void clear() { hebiCommandClearEnum(internal_, field_); }

      private:
        HebiCommandPtr const internal_;
        CommandEnumField const field_;

        HEBI_DISABLE_COPY_MOVE(EnumField)
    };

    /// \brief A message field for interfacing with an LED.
    class LedField final
    {
      public:
        #ifndef DOXYGEN_OMIT_INTERNAL
        LedField(HebiCommandPtr internal, CommandLedField field);
        #endif // DOXYGEN_OMIT_INTERNAL
        /// \brief Returns true if the LED color is set, and false otherwise.
        bool hasColor() const;
        /// \brief Returns true if the message indicates that the module should
        /// resume control of the LED.
        ///
        /// Note: a return value of @c false can indicate either an override
        /// command (if and only if @c hasColor() returns @c true), or no
        /// information about the LED (i.e., the module should maintain it's
        /// current state regarding the LED).
        bool hasModuleControl() const;
        /// \brief Returns the led color.
        Color getColor() const;
        /// \brief Commands a color that overrides the module's control of the
        /// LED.
        void setOverrideColor(const Color& new_color);
        /// \brief Sets the module to regain control of the LED.
        void setModuleControl();
        /// \brief Removes any currently set value for this field, so that the
        /// module maintains its previous state of LED control/color (i.e., does
        /// not have an override color command or an explicit 'module control'
        /// command).
        void clear();

      private:
        HebiCommandPtr const internal_;
        CommandLedField const field_;

        HEBI_DISABLE_COPY_MOVE(LedField)
    };

    /// Module settings that are typically changed at a slower rate.
    class Settings final
    {
      // Note: this is 'protected' instead of 'private' for easier use with Doxygen
      protected:
        /// Actuator-specific settings, such as controller gains.
        class Actuator final
        {
          // Note: this is 'protected' instead of 'private' for easier use with Doxygen
          protected:
            /// Controller gains for the position PID loop.
            class PositionGains final
            {
              public:
                #ifndef DOXYGEN_OMIT_INTERNAL
                PositionGains(HebiCommandPtr internal)
                  : internal_(internal),
                    position_kp_(internal, CommandFloatPositionKp),
                    position_ki_(internal, CommandFloatPositionKi),
                    position_kd_(internal, CommandFloatPositionKd),
                    position_feed_forward_(internal, CommandFloatPositionFeedForward),
                    position_dead_zone_(internal, CommandFloatPositionDeadZone),
                    position_i_clamp_(internal, CommandFloatPositionIClamp),
                    position_punch_(internal, CommandFloatPositionPunch),
                    position_min_target_(internal, CommandFloatPositionMinTarget),
                    position_max_target_(internal, CommandFloatPositionMaxTarget),
                    position_target_lowpass_(internal, CommandFloatPositionTargetLowpass),
                    position_min_output_(internal, CommandFloatPositionMinOutput),
                    position_max_output_(internal, CommandFloatPositionMaxOutput),
                    position_output_lowpass_(internal, CommandFloatPositionOutputLowpass),
                    position_d_on_error_(internal, CommandBoolPositionDOnError)
                {
                }
                #endif // DOXYGEN_OMIT_INTERNAL
            
                // With all submessage and field getters: Note that the returned reference
                // should not be used after the lifetime of this parent.
            
                // Subfields ----------------
            
                /// Proportional PID gain for position
                FloatField& positionKp() { return position_kp_; }
                /// Integral PID gain for position
                FloatField& positionKi() { return position_ki_; }
                /// Derivative PID gain for position
                FloatField& positionKd() { return position_kd_; }
                /// Feed forward term for position (this term is multiplied by the target and added to the output).
                FloatField& positionFeedForward() { return position_feed_forward_; }
                /// Error values within +/- this value from zero are treated as zero (in terms of computed proportional output, input to numerical derivative, and accumulated integral error).
                FloatField& positionDeadZone() { return position_dead_zone_; }
                /// Maximum allowed value for the output of the integral component of the PID loop; the integrated error is not allowed to exceed value that will generate this number.
                FloatField& positionIClamp() { return position_i_clamp_; }
                /// Constant offset to the position PID output outside of the deadzone; it is added when the error is positive and subtracted when it is negative.
                FloatField& positionPunch() { return position_punch_; }
                /// Minimum allowed value for input to the PID controller
                FloatField& positionMinTarget() { return position_min_target_; }
                /// Maximum allowed value for input to the PID controller
                FloatField& positionMaxTarget() { return position_max_target_; }
                /// A simple lowpass filter applied to the target set point; needs to be between 0 and 1.  At each timestep: x_t = x_t * a + x_{t-1} * (1 - a).
                FloatField& positionTargetLowpass() { return position_target_lowpass_; }
                /// Output from the PID controller is limited to a minimum of this value.
                FloatField& positionMinOutput() { return position_min_output_; }
                /// Output from the PID controller is limited to a maximum of this value.
                FloatField& positionMaxOutput() { return position_max_output_; }
                /// A simple lowpass filter applied to the controller output; needs to be between 0 and 1.  At each timestep: x_t = x_t * a + x_{t-1} * (1 - a).
                FloatField& positionOutputLowpass() { return position_output_lowpass_; }
                /// Controls whether the Kd term uses the "derivative of error" or "derivative of measurement."  When the setpoints have step inputs or are noisy, setting this to @c false can eliminate corresponding spikes or noise in the output.
                BoolField& positionDOnError() { return position_d_on_error_; }
            
              private:
                HebiCommandPtr const internal_;
            
                FloatField position_kp_;
                FloatField position_ki_;
                FloatField position_kd_;
                FloatField position_feed_forward_;
                FloatField position_dead_zone_;
                FloatField position_i_clamp_;
                FloatField position_punch_;
                FloatField position_min_target_;
                FloatField position_max_target_;
                FloatField position_target_lowpass_;
                FloatField position_min_output_;
                FloatField position_max_output_;
                FloatField position_output_lowpass_;
                BoolField position_d_on_error_;
            
                HEBI_DISABLE_COPY_MOVE(PositionGains)
            };
        
            /// Controller gains for the velocity PID loop.
            class VelocityGains final
            {
              public:
                #ifndef DOXYGEN_OMIT_INTERNAL
                VelocityGains(HebiCommandPtr internal)
                  : internal_(internal),
                    velocity_kp_(internal, CommandFloatVelocityKp),
                    velocity_ki_(internal, CommandFloatVelocityKi),
                    velocity_kd_(internal, CommandFloatVelocityKd),
                    velocity_feed_forward_(internal, CommandFloatVelocityFeedForward),
                    velocity_dead_zone_(internal, CommandFloatVelocityDeadZone),
                    velocity_i_clamp_(internal, CommandFloatVelocityIClamp),
                    velocity_punch_(internal, CommandFloatVelocityPunch),
                    velocity_min_target_(internal, CommandFloatVelocityMinTarget),
                    velocity_max_target_(internal, CommandFloatVelocityMaxTarget),
                    velocity_target_lowpass_(internal, CommandFloatVelocityTargetLowpass),
                    velocity_min_output_(internal, CommandFloatVelocityMinOutput),
                    velocity_max_output_(internal, CommandFloatVelocityMaxOutput),
                    velocity_output_lowpass_(internal, CommandFloatVelocityOutputLowpass),
                    velocity_d_on_error_(internal, CommandBoolVelocityDOnError)
                {
                }
                #endif // DOXYGEN_OMIT_INTERNAL
            
                // With all submessage and field getters: Note that the returned reference
                // should not be used after the lifetime of this parent.
            
                // Subfields ----------------
            
                /// Proportional PID gain for velocity
                FloatField& velocityKp() { return velocity_kp_; }
                /// Integral PID gain for velocity
                FloatField& velocityKi() { return velocity_ki_; }
                /// Derivative PID gain for velocity
                FloatField& velocityKd() { return velocity_kd_; }
                /// Feed forward term for velocity (this term is multiplied by the target and added to the output).
                FloatField& velocityFeedForward() { return velocity_feed_forward_; }
                /// Error values within +/- this value from zero are treated as zero (in terms of computed proportional output, input to numerical derivative, and accumulated integral error).
                FloatField& velocityDeadZone() { return velocity_dead_zone_; }
                /// Maximum allowed value for the output of the integral component of the PID loop; the integrated error is not allowed to exceed value that will generate this number.
                FloatField& velocityIClamp() { return velocity_i_clamp_; }
                /// Constant offset to the velocity PID output outside of the deadzone; it is added when the error is positive and subtracted when it is negative.
                FloatField& velocityPunch() { return velocity_punch_; }
                /// Minimum allowed value for input to the PID controller
                FloatField& velocityMinTarget() { return velocity_min_target_; }
                /// Maximum allowed value for input to the PID controller
                FloatField& velocityMaxTarget() { return velocity_max_target_; }
                /// A simple lowpass filter applied to the target set point; needs to be between 0 and 1.  At each timestep: x_t = x_t * a + x_{t-1} * (1 - a).
                FloatField& velocityTargetLowpass() { return velocity_target_lowpass_; }
                /// Output from the PID controller is limited to a minimum of this value.
                FloatField& velocityMinOutput() { return velocity_min_output_; }
                /// Output from the PID controller is limited to a maximum of this value.
                FloatField& velocityMaxOutput() { return velocity_max_output_; }
                /// A simple lowpass filter applied to the controller output; needs to be between 0 and 1.  At each timestep: x_t = x_t * a + x_{t-1} * (1 - a).
                FloatField& velocityOutputLowpass() { return velocity_output_lowpass_; }
                /// Controls whether the Kd term uses the "derivative of error" or "derivative of measurement."  When the setpoints have step inputs or are noisy, setting this to @c false can eliminate corresponding spikes or noise in the output.
                BoolField& velocityDOnError() { return velocity_d_on_error_; }
            
              private:
                HebiCommandPtr const internal_;
            
                FloatField velocity_kp_;
                FloatField velocity_ki_;
                FloatField velocity_kd_;
                FloatField velocity_feed_forward_;
                FloatField velocity_dead_zone_;
                FloatField velocity_i_clamp_;
                FloatField velocity_punch_;
                FloatField velocity_min_target_;
                FloatField velocity_max_target_;
                FloatField velocity_target_lowpass_;
                FloatField velocity_min_output_;
                FloatField velocity_max_output_;
                FloatField velocity_output_lowpass_;
                BoolField velocity_d_on_error_;
            
                HEBI_DISABLE_COPY_MOVE(VelocityGains)
            };
        
            /// Controller gains for the torque PID loop.
            class TorqueGains final
            {
              public:
                #ifndef DOXYGEN_OMIT_INTERNAL
                TorqueGains(HebiCommandPtr internal)
                  : internal_(internal),
                    torque_kp_(internal, CommandFloatTorqueKp),
                    torque_ki_(internal, CommandFloatTorqueKi),
                    torque_kd_(internal, CommandFloatTorqueKd),
                    torque_feed_forward_(internal, CommandFloatTorqueFeedForward),
                    torque_dead_zone_(internal, CommandFloatTorqueDeadZone),
                    torque_i_clamp_(internal, CommandFloatTorqueIClamp),
                    torque_punch_(internal, CommandFloatTorquePunch),
                    torque_min_target_(internal, CommandFloatTorqueMinTarget),
                    torque_max_target_(internal, CommandFloatTorqueMaxTarget),
                    torque_target_lowpass_(internal, CommandFloatTorqueTargetLowpass),
                    torque_min_output_(internal, CommandFloatTorqueMinOutput),
                    torque_max_output_(internal, CommandFloatTorqueMaxOutput),
                    torque_output_lowpass_(internal, CommandFloatTorqueOutputLowpass),
                    torque_d_on_error_(internal, CommandBoolTorqueDOnError)
                {
                }
                #endif // DOXYGEN_OMIT_INTERNAL
            
                // With all submessage and field getters: Note that the returned reference
                // should not be used after the lifetime of this parent.
            
                // Subfields ----------------
            
                /// Proportional PID gain for torque
                FloatField& torqueKp() { return torque_kp_; }
                /// Integral PID gain for torque
                FloatField& torqueKi() { return torque_ki_; }
                /// Derivative PID gain for torque
                FloatField& torqueKd() { return torque_kd_; }
                /// Feed forward term for torque (this term is multiplied by the target and added to the output).
                FloatField& torqueFeedForward() { return torque_feed_forward_; }
                /// Error values within +/- this value from zero are treated as zero (in terms of computed proportional output, input to numerical derivative, and accumulated integral error).
                FloatField& torqueDeadZone() { return torque_dead_zone_; }
                /// Maximum allowed value for the output of the integral component of the PID loop; the integrated error is not allowed to exceed value that will generate this number.
                FloatField& torqueIClamp() { return torque_i_clamp_; }
                /// Constant offset to the torque PID output outside of the deadzone; it is added when the error is positive and subtracted when it is negative.
                FloatField& torquePunch() { return torque_punch_; }
                /// Minimum allowed value for input to the PID controller
                FloatField& torqueMinTarget() { return torque_min_target_; }
                /// Maximum allowed value for input to the PID controller
                FloatField& torqueMaxTarget() { return torque_max_target_; }
                /// A simple lowpass filter applied to the target set point; needs to be between 0 and 1.  At each timestep: x_t = x_t * a + x_{t-1} * (1 - a).
                FloatField& torqueTargetLowpass() { return torque_target_lowpass_; }
                /// Output from the PID controller is limited to a minimum of this value.
                FloatField& torqueMinOutput() { return torque_min_output_; }
                /// Output from the PID controller is limited to a maximum of this value.
                FloatField& torqueMaxOutput() { return torque_max_output_; }
                /// A simple lowpass filter applied to the controller output; needs to be between 0 and 1.  At each timestep: x_t = x_t * a + x_{t-1} * (1 - a).
                FloatField& torqueOutputLowpass() { return torque_output_lowpass_; }
                /// Controls whether the Kd term uses the "derivative of error" or "derivative of measurement."  When the setpoints have step inputs or are noisy, setting this to @c false can eliminate corresponding spikes or noise in the output.
                BoolField& torqueDOnError() { return torque_d_on_error_; }
            
              private:
                HebiCommandPtr const internal_;
            
                FloatField torque_kp_;
                FloatField torque_ki_;
                FloatField torque_kd_;
                FloatField torque_feed_forward_;
                FloatField torque_dead_zone_;
                FloatField torque_i_clamp_;
                FloatField torque_punch_;
                FloatField torque_min_target_;
                FloatField torque_max_target_;
                FloatField torque_target_lowpass_;
                FloatField torque_min_output_;
                FloatField torque_max_output_;
                FloatField torque_output_lowpass_;
                BoolField torque_d_on_error_;
            
                HEBI_DISABLE_COPY_MOVE(TorqueGains)
            };
        
          public:
            #ifndef DOXYGEN_OMIT_INTERNAL
            Actuator(HebiCommandPtr internal)
              : internal_(internal),
                position_gains_(internal),
                velocity_gains_(internal),
                torque_gains_(internal),
                spring_constant_(internal, CommandFloatSpringConstant),
                control_strategy_(internal, CommandEnumControlStrategy)
            {
            }
            #endif // DOXYGEN_OMIT_INTERNAL
        
            // With all submessage and field getters: Note that the returned reference
            // should not be used after the lifetime of this parent.
        
            // Submessages ----------------
        
            /// Controller gains for the position PID loop.
            PositionGains& positionGains() { return position_gains_; }
            /// Controller gains for the velocity PID loop.
            VelocityGains& velocityGains() { return velocity_gains_; }
            /// Controller gains for the torque PID loop.
            TorqueGains& torqueGains() { return torque_gains_; }
        
            // Subfields ----------------
        
            /// The spring constant of the module.
            FloatField& springConstant() { return spring_constant_; }
            /// How the position, velocity, and torque PID loops are connected in order to control motor PWM.
            EnumField<ControlStrategy>& controlStrategy() { return control_strategy_; }
        
          private:
            HebiCommandPtr const internal_;
        
            PositionGains position_gains_;
            VelocityGains velocity_gains_;
            TorqueGains torque_gains_;
        
            FloatField spring_constant_;
            EnumField<ControlStrategy> control_strategy_;
        
            HEBI_DISABLE_COPY_MOVE(Actuator)
        };
    
      public:
        #ifndef DOXYGEN_OMIT_INTERNAL
        Settings(HebiCommandPtr internal)
          : internal_(internal),
            actuator_(internal),
            name_(internal, CommandStringName),
            family_(internal, CommandStringFamily),
            save_current_settings_(internal, CommandFlagSaveCurrentSettings)
        {
        }
        #endif // DOXYGEN_OMIT_INTERNAL
    
        // With all submessage and field getters: Note that the returned reference
        // should not be used after the lifetime of this parent.
    
        // Submessages ----------------
    
        /// Actuator-specific settings, such as controller gains.
        Actuator& actuator() { return actuator_; }
    
        // Subfields ----------------
    
        /// Sets the name for this module. Name must be null-terminated character string for the name; must be <= 20 characters.
        StringField& name() { return name_; }
        /// Sets the family for this module. Name must be null-terminated character string for the family; must be <= 20 characters.
        StringField& family() { return family_; }
        /// Indicates if the module should save the current values of all of its settings.
        FlagField& saveCurrentSettings() { return save_current_settings_; }
    
      private:
        HebiCommandPtr const internal_;
    
        Actuator actuator_;
    
        StringField name_;
        StringField family_;
        FlagField save_current_settings_;
    
        HEBI_DISABLE_COPY_MOVE(Settings)
    };

    /// Actuator-specific commands.
    class Actuator final
    {
      public:
        #ifndef DOXYGEN_OMIT_INTERNAL
        Actuator(HebiCommandPtr internal)
          : internal_(internal),
            velocity_(internal, CommandFloatVelocity),
            torque_(internal, CommandFloatTorque),
            position_(internal, CommandHighResAnglePosition)
        {
        }
        #endif // DOXYGEN_OMIT_INTERNAL
    
        // With all submessage and field getters: Note that the returned reference
        // should not be used after the lifetime of this parent.
    
        // Subfields ----------------
    
        /// Velocity of the module output (post-spring), in radians/second.
        FloatField& velocity() { return velocity_; }
        /// Torque at the module output, in N * m.
        FloatField& torque() { return torque_; }
        /// Position of the module output (post-spring), in radians.
        HighResAngleField& position() { return position_; }
    
      private:
        HebiCommandPtr const internal_;
    
        FloatField velocity_;
        FloatField torque_;
        HighResAngleField position_;
    
        HEBI_DISABLE_COPY_MOVE(Actuator)
    };

  public:
    #ifndef DOXYGEN_OMIT_INTERNAL
    /**
     * C-style command object.
     * NOTE: this should not be used except by internal library functions!
     */
    HebiCommandPtr const internal_;
    #endif // DOXYGEN_OMIT_INTERNAL

  private:
    /**
     * True if this object is responsible for creating and destroying the
     * underlying C pointer; false otherwise.
     */
    bool manage_pointer_lifetime_;

  public:
    /**
     * \brief Create a new command object for a single module.
     */
    Command();
    #ifndef DOXYGEN_OMIT_INTERNAL
    /**
     * \brief Wraps an existing C-style command object; object lifetime is assumed to
     * be managed by the caller.
     *
     * NOTE: this should not be used except by internal library functions!
     */
    Command(HebiCommandPtr command);
    #endif // DOXYGEN_OMIT_INTERNAL
    /**
     * \brief Move constructor (necessary for containment in STL template classes)
     */
    Command(Command&& other);

    /**
     * \brief Destructor cleans up command object as necessary.
     */
    virtual ~Command() noexcept; /* annotating specified destructor as noexcept is best-practice */

    // With all submessage and field getters: Note that the returned reference
    // should not be used after the lifetime of this parent.

    // Submessages -------------------------------------------------------------

    /// Module settings that are typically changed at a slower rate.
    Settings& settings();
    /// Actuator-specific commands.
    Actuator& actuator();

    // Subfields -------------------------------------------------------------

    #ifndef DOXYGEN_OMIT_INTERNAL
    /// Values for internal debug functions (channel 1-9 available).
    NumberedFloatField& debug();
    #endif // DOXYGEN_OMIT_INTERNAL
    /// The module's LED.
    LedField& led();

  private:
    Settings settings_;
    Actuator actuator_;

    NumberedFloatField debug_;
    LedField led_;

    /**
     * Disable copy constructor/assignment operators
     */
    HEBI_DISABLE_COPY(Command)

    /* Disable move assigment operator. */
    Command& operator= (const Command&& other) = delete;
};

} // namespace hebi

#endif // COMMAND_HPP
