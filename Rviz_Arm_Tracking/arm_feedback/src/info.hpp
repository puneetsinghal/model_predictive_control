#ifndef INFO_HPP
#define INFO_HPP

#include "hebi_info.h"
#include "color.hpp"
#include <string>
#include "util.hpp"

namespace hebi {

/// \brief Info objects have various fields representing the module state; which
/// fields are populated depends on the module type and various other settings.
/// 
/// This object has a hierarchical structure -- there are some direct general-purpose
/// fields at the top level, and many more specific fields contained in different
/// nested subobjects.
/// 
/// The subobjects contain references to the parent info object, and so should not be
/// used after the parent object has been destroyed.
/// 
/// The fields in the info object are typed; generally, these are optional-style
/// read-only fields (i.e., have the concept of has/get), although the return types
/// and exact interface vary slightly between fields. Where appropriate, the explicit
/// bool operator has been overridden so that you can shortcut @c if(field.has()) by
/// calling @c if(field).
/// 
/// Although this header file can be used to look at the hierarchy of the messages,
/// in general the online documentation at apidocs.hebi.us presents this information.
/// in a more readable form.
class Info final
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
        FloatField(HebiInfoPtr internal, InfoFloatField field);
        #endif // DOXYGEN_OMIT_INTERNAL
        /// \brief Allows casting to a bool to check if the field has a value
        /// without directly calling @c has().
        ///
        /// This can be used as in the following (assuming 'parent' is a parent message,
        /// and this field is called 'myField')
        /// \code{.cpp}
        /// Info::FloatField& f = parent.myField();
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

      private:
        HebiInfoPtr const internal_;
        InfoFloatField const field_;

        HEBI_DISABLE_COPY_MOVE(FloatField)
    };
    /// \brief A message field representable by a bool value.
    class BoolField final
    {
      public:
        #ifndef DOXYGEN_OMIT_INTERNAL
        BoolField(HebiInfoPtr internal, InfoBoolField field);
        #endif // DOXYGEN_OMIT_INTERNAL
        /// \brief True if (and only if) the field has a value.
        bool has() const;
        /// \brief If the field has a value, returns that value; otherwise,
        /// returns false.
        bool get() const;

      private:
        HebiInfoPtr const internal_;
        InfoBoolField const field_;

        HEBI_DISABLE_COPY_MOVE(BoolField)
    };

    /// \brief A message field representable by a std::string.
    class StringField final
    {
      public:
        #ifndef DOXYGEN_OMIT_INTERNAL
        StringField(HebiInfoPtr internal, InfoStringField field);
        #endif // DOXYGEN_OMIT_INTERNAL
        /// \brief Allows casting to a bool to check if the field has a value
        /// without directly calling @c has().
        ///
        /// This can be used as in the following (assuming 'parent' is a parent message,
        /// and this field is called 'myField')
        /// \code{.cpp}
        /// Info::StringField& f = parent.myField();
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

      private:
        HebiInfoPtr const internal_;
        InfoStringField const field_;

        HEBI_DISABLE_COPY_MOVE(StringField)
    };

    /// \brief A two-state message field (either set/true or cleared/false).
    class FlagField final
    {
      public:
        #ifndef DOXYGEN_OMIT_INTERNAL
        FlagField(HebiInfoPtr internal, InfoFlagField field);
        #endif // DOXYGEN_OMIT_INTERNAL
        /// \brief Allows casting to a bool to check if the flag is set without
        /// directly calling @c has().
        ///
        /// This can be used as in the following (assuming 'parent' is a parent message,
        /// and this field is called 'myField')
        /// \code{.cpp}
        /// Info::FlagField& f = parent.myField();
        /// if (f)
        ///   std::cout << "Field has value: " << f.get() << std::endl;
        /// else
        ///   std::cout << "Field has no value!" << std::endl;
        /// \endcode
        explicit operator bool() const;
        /// \brief Returns @c true if the flag is set, false if it is cleared.
        bool has() const;

      private:
        HebiInfoPtr const internal_;
        InfoFlagField const field_;

        HEBI_DISABLE_COPY_MOVE(FlagField)
    };

    /// \brief A message field representable by an enum of a given type.
    template <class T>
    class EnumField final
    {
      public:
        #ifndef DOXYGEN_OMIT_INTERNAL
        EnumField(HebiInfoPtr internal, InfoEnumField field)
          : internal_(internal), field_(field) {}
        #endif // DOXYGEN_OMIT_INTERNAL
        /// \brief Allows casting to a bool to check if the field has a value
        /// without directly calling @c has().
        ///
        /// This can be used as in the following (assuming 'parent' is a parent message,
        /// and this field is called 'myField')
        /// \code{.cpp}
        /// Info::EnumField& f = parent.myField();
        /// if (f)
        ///   std::cout << "Field has value: " << f.get() << std::endl;
        /// else
        ///   std::cout << "Field has no value!" << std::endl;
        /// \endcode
        explicit operator bool() const { return has(); }
        /// \brief True if (and only if) the field has a value.
        bool has() const { return (hebiInfoHasEnum(internal_, field_) == 1); }
        /// \brief If the field has a value, returns that value; otherwise,
        /// returns a default.
        T get() const { return (T)hebiInfoGetEnum(internal_, field_); }

      private:
        HebiInfoPtr const internal_;
        InfoEnumField const field_;

        HEBI_DISABLE_COPY_MOVE(EnumField)
    };

    /// \brief A message field for interfacing with an LED.
    class LedField final
    {
      public:
        #ifndef DOXYGEN_OMIT_INTERNAL
        LedField(HebiInfoPtr internal, InfoLedField field);
        #endif // DOXYGEN_OMIT_INTERNAL
        /// \brief Allows casting to a bool to check if the LED color is set
        /// without directly calling @c hasColor().
        ///
        /// This can be used as in the following (assuming 'parent' is a parent message,
        /// and this field is called 'myField')
        /// \code{.cpp}
        /// Info::LedField& f = parent.myField();
        /// if (f)
        ///   std::cout << "Field has color!" << std::endl;
        /// else
        ///   std::cout << "Field has no value!" << std::endl;
        /// \endcode
        explicit operator bool() const { return hasColor(); }
        /// \brief Returns true if the LED color is set, and false otherwise.
        bool hasColor() const;
        /// \brief Returns the led color.
        Color getColor() const;

      private:
        HebiInfoPtr const internal_;
        InfoLedField const field_;

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
                PositionGains(HebiInfoPtr internal)
                  : internal_(internal),
                    position_kp_(internal, InfoFloatPositionKp),
                    position_ki_(internal, InfoFloatPositionKi),
                    position_kd_(internal, InfoFloatPositionKd),
                    position_feed_forward_(internal, InfoFloatPositionFeedForward),
                    position_dead_zone_(internal, InfoFloatPositionDeadZone),
                    position_i_clamp_(internal, InfoFloatPositionIClamp),
                    position_punch_(internal, InfoFloatPositionPunch),
                    position_min_target_(internal, InfoFloatPositionMinTarget),
                    position_max_target_(internal, InfoFloatPositionMaxTarget),
                    position_target_lowpass_(internal, InfoFloatPositionTargetLowpass),
                    position_min_output_(internal, InfoFloatPositionMinOutput),
                    position_max_output_(internal, InfoFloatPositionMaxOutput),
                    position_output_lowpass_(internal, InfoFloatPositionOutputLowpass),
                    position_d_on_error_(internal, InfoBoolPositionDOnError)
                {
                }
                #endif // DOXYGEN_OMIT_INTERNAL
            
                // With all submessage and field getters: Note that the returned reference
                // should not be used after the lifetime of this parent.
            
                // Subfields ----------------
            
                /// Proportional PID gain for position
                const FloatField& positionKp() const { return position_kp_; }
                /// Integral PID gain for position
                const FloatField& positionKi() const { return position_ki_; }
                /// Derivative PID gain for position
                const FloatField& positionKd() const { return position_kd_; }
                /// Feed forward term for position (this term is multiplied by the target and added to the output).
                const FloatField& positionFeedForward() const { return position_feed_forward_; }
                /// Error values within +/- this value from zero are treated as zero (in terms of computed proportional output, input to numerical derivative, and accumulated integral error).
                const FloatField& positionDeadZone() const { return position_dead_zone_; }
                /// Maximum allowed value for the output of the integral component of the PID loop; the integrated error is not allowed to exceed value that will generate this number.
                const FloatField& positionIClamp() const { return position_i_clamp_; }
                /// Constant offset to the position PID output outside of the deadzone; it is added when the error is positive and subtracted when it is negative.
                const FloatField& positionPunch() const { return position_punch_; }
                /// Minimum allowed value for input to the PID controller
                const FloatField& positionMinTarget() const { return position_min_target_; }
                /// Maximum allowed value for input to the PID controller
                const FloatField& positionMaxTarget() const { return position_max_target_; }
                /// A simple lowpass filter applied to the target set point; needs to be between 0 and 1.  At each timestep: x_t = x_t * a + x_{t-1} * (1 - a).
                const FloatField& positionTargetLowpass() const { return position_target_lowpass_; }
                /// Output from the PID controller is limited to a minimum of this value.
                const FloatField& positionMinOutput() const { return position_min_output_; }
                /// Output from the PID controller is limited to a maximum of this value.
                const FloatField& positionMaxOutput() const { return position_max_output_; }
                /// A simple lowpass filter applied to the controller output; needs to be between 0 and 1.  At each timestep: x_t = x_t * a + x_{t-1} * (1 - a).
                const FloatField& positionOutputLowpass() const { return position_output_lowpass_; }
                /// Controls whether the Kd term uses the "derivative of error" or "derivative of measurement."  When the setpoints have step inputs or are noisy, setting this to @c false can eliminate corresponding spikes or noise in the output.
                const BoolField& positionDOnError() const { return position_d_on_error_; }
            
              private:
                HebiInfoPtr const internal_;
            
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
                VelocityGains(HebiInfoPtr internal)
                  : internal_(internal),
                    velocity_kp_(internal, InfoFloatVelocityKp),
                    velocity_ki_(internal, InfoFloatVelocityKi),
                    velocity_kd_(internal, InfoFloatVelocityKd),
                    velocity_feed_forward_(internal, InfoFloatVelocityFeedForward),
                    velocity_dead_zone_(internal, InfoFloatVelocityDeadZone),
                    velocity_i_clamp_(internal, InfoFloatVelocityIClamp),
                    velocity_punch_(internal, InfoFloatVelocityPunch),
                    velocity_min_target_(internal, InfoFloatVelocityMinTarget),
                    velocity_max_target_(internal, InfoFloatVelocityMaxTarget),
                    velocity_target_lowpass_(internal, InfoFloatVelocityTargetLowpass),
                    velocity_min_output_(internal, InfoFloatVelocityMinOutput),
                    velocity_max_output_(internal, InfoFloatVelocityMaxOutput),
                    velocity_output_lowpass_(internal, InfoFloatVelocityOutputLowpass),
                    velocity_d_on_error_(internal, InfoBoolVelocityDOnError)
                {
                }
                #endif // DOXYGEN_OMIT_INTERNAL
            
                // With all submessage and field getters: Note that the returned reference
                // should not be used after the lifetime of this parent.
            
                // Subfields ----------------
            
                /// Proportional PID gain for velocity
                const FloatField& velocityKp() const { return velocity_kp_; }
                /// Integral PID gain for velocity
                const FloatField& velocityKi() const { return velocity_ki_; }
                /// Derivative PID gain for velocity
                const FloatField& velocityKd() const { return velocity_kd_; }
                /// Feed forward term for velocity (this term is multiplied by the target and added to the output).
                const FloatField& velocityFeedForward() const { return velocity_feed_forward_; }
                /// Error values within +/- this value from zero are treated as zero (in terms of computed proportional output, input to numerical derivative, and accumulated integral error).
                const FloatField& velocityDeadZone() const { return velocity_dead_zone_; }
                /// Maximum allowed value for the output of the integral component of the PID loop; the integrated error is not allowed to exceed value that will generate this number.
                const FloatField& velocityIClamp() const { return velocity_i_clamp_; }
                /// Constant offset to the velocity PID output outside of the deadzone; it is added when the error is positive and subtracted when it is negative.
                const FloatField& velocityPunch() const { return velocity_punch_; }
                /// Minimum allowed value for input to the PID controller
                const FloatField& velocityMinTarget() const { return velocity_min_target_; }
                /// Maximum allowed value for input to the PID controller
                const FloatField& velocityMaxTarget() const { return velocity_max_target_; }
                /// A simple lowpass filter applied to the target set point; needs to be between 0 and 1.  At each timestep: x_t = x_t * a + x_{t-1} * (1 - a).
                const FloatField& velocityTargetLowpass() const { return velocity_target_lowpass_; }
                /// Output from the PID controller is limited to a minimum of this value.
                const FloatField& velocityMinOutput() const { return velocity_min_output_; }
                /// Output from the PID controller is limited to a maximum of this value.
                const FloatField& velocityMaxOutput() const { return velocity_max_output_; }
                /// A simple lowpass filter applied to the controller output; needs to be between 0 and 1.  At each timestep: x_t = x_t * a + x_{t-1} * (1 - a).
                const FloatField& velocityOutputLowpass() const { return velocity_output_lowpass_; }
                /// Controls whether the Kd term uses the "derivative of error" or "derivative of measurement."  When the setpoints have step inputs or are noisy, setting this to @c false can eliminate corresponding spikes or noise in the output.
                const BoolField& velocityDOnError() const { return velocity_d_on_error_; }
            
              private:
                HebiInfoPtr const internal_;
            
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
                TorqueGains(HebiInfoPtr internal)
                  : internal_(internal),
                    torque_kp_(internal, InfoFloatTorqueKp),
                    torque_ki_(internal, InfoFloatTorqueKi),
                    torque_kd_(internal, InfoFloatTorqueKd),
                    torque_feed_forward_(internal, InfoFloatTorqueFeedForward),
                    torque_dead_zone_(internal, InfoFloatTorqueDeadZone),
                    torque_i_clamp_(internal, InfoFloatTorqueIClamp),
                    torque_punch_(internal, InfoFloatTorquePunch),
                    torque_min_target_(internal, InfoFloatTorqueMinTarget),
                    torque_max_target_(internal, InfoFloatTorqueMaxTarget),
                    torque_target_lowpass_(internal, InfoFloatTorqueTargetLowpass),
                    torque_min_output_(internal, InfoFloatTorqueMinOutput),
                    torque_max_output_(internal, InfoFloatTorqueMaxOutput),
                    torque_output_lowpass_(internal, InfoFloatTorqueOutputLowpass),
                    torque_d_on_error_(internal, InfoBoolTorqueDOnError)
                {
                }
                #endif // DOXYGEN_OMIT_INTERNAL
            
                // With all submessage and field getters: Note that the returned reference
                // should not be used after the lifetime of this parent.
            
                // Subfields ----------------
            
                /// Proportional PID gain for torque
                const FloatField& torqueKp() const { return torque_kp_; }
                /// Integral PID gain for torque
                const FloatField& torqueKi() const { return torque_ki_; }
                /// Derivative PID gain for torque
                const FloatField& torqueKd() const { return torque_kd_; }
                /// Feed forward term for torque (this term is multiplied by the target and added to the output).
                const FloatField& torqueFeedForward() const { return torque_feed_forward_; }
                /// Error values within +/- this value from zero are treated as zero (in terms of computed proportional output, input to numerical derivative, and accumulated integral error).
                const FloatField& torqueDeadZone() const { return torque_dead_zone_; }
                /// Maximum allowed value for the output of the integral component of the PID loop; the integrated error is not allowed to exceed value that will generate this number.
                const FloatField& torqueIClamp() const { return torque_i_clamp_; }
                /// Constant offset to the torque PID output outside of the deadzone; it is added when the error is positive and subtracted when it is negative.
                const FloatField& torquePunch() const { return torque_punch_; }
                /// Minimum allowed value for input to the PID controller
                const FloatField& torqueMinTarget() const { return torque_min_target_; }
                /// Maximum allowed value for input to the PID controller
                const FloatField& torqueMaxTarget() const { return torque_max_target_; }
                /// A simple lowpass filter applied to the target set point; needs to be between 0 and 1.  At each timestep: x_t = x_t * a + x_{t-1} * (1 - a).
                const FloatField& torqueTargetLowpass() const { return torque_target_lowpass_; }
                /// Output from the PID controller is limited to a minimum of this value.
                const FloatField& torqueMinOutput() const { return torque_min_output_; }
                /// Output from the PID controller is limited to a maximum of this value.
                const FloatField& torqueMaxOutput() const { return torque_max_output_; }
                /// A simple lowpass filter applied to the controller output; needs to be between 0 and 1.  At each timestep: x_t = x_t * a + x_{t-1} * (1 - a).
                const FloatField& torqueOutputLowpass() const { return torque_output_lowpass_; }
                /// Controls whether the Kd term uses the "derivative of error" or "derivative of measurement."  When the setpoints have step inputs or are noisy, setting this to @c false can eliminate corresponding spikes or noise in the output.
                const BoolField& torqueDOnError() const { return torque_d_on_error_; }
            
              private:
                HebiInfoPtr const internal_;
            
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
            Actuator(HebiInfoPtr internal)
              : internal_(internal),
                position_gains_(internal),
                velocity_gains_(internal),
                torque_gains_(internal),
                spring_constant_(internal, InfoFloatSpringConstant),
                control_strategy_(internal, InfoEnumControlStrategy)
            {
            }
            #endif // DOXYGEN_OMIT_INTERNAL
        
            // With all submessage and field getters: Note that the returned reference
            // should not be used after the lifetime of this parent.
        
            // Submessages ----------------
        
            /// Controller gains for the position PID loop.
            const PositionGains& positionGains() const { return position_gains_; }
            /// Controller gains for the velocity PID loop.
            const VelocityGains& velocityGains() const { return velocity_gains_; }
            /// Controller gains for the torque PID loop.
            const TorqueGains& torqueGains() const { return torque_gains_; }
        
            // Subfields ----------------
        
            /// The spring constant of the module.
            const FloatField& springConstant() const { return spring_constant_; }
            /// How the position, velocity, and torque PID loops are connected in order to control motor PWM.
            const EnumField<ControlStrategy>& controlStrategy() const { return control_strategy_; }
        
          private:
            HebiInfoPtr const internal_;
        
            PositionGains position_gains_;
            VelocityGains velocity_gains_;
            TorqueGains torque_gains_;
        
            FloatField spring_constant_;
            EnumField<ControlStrategy> control_strategy_;
        
            HEBI_DISABLE_COPY_MOVE(Actuator)
        };
    
      public:
        #ifndef DOXYGEN_OMIT_INTERNAL
        Settings(HebiInfoPtr internal)
          : internal_(internal),
            actuator_(internal),
            name_(internal, InfoStringName),
            family_(internal, InfoStringFamily),
            save_current_settings_(internal, InfoFlagSaveCurrentSettings)
        {
        }
        #endif // DOXYGEN_OMIT_INTERNAL
    
        // With all submessage and field getters: Note that the returned reference
        // should not be used after the lifetime of this parent.
    
        // Submessages ----------------
    
        /// Actuator-specific settings, such as controller gains.
        const Actuator& actuator() const { return actuator_; }
    
        // Subfields ----------------
    
        /// Sets the name for this module. Name must be null-terminated character string for the name; must be <= 20 characters.
        const StringField& name() const { return name_; }
        /// Sets the family for this module. Name must be null-terminated character string for the family; must be <= 20 characters.
        const StringField& family() const { return family_; }
        /// Indicates if the module should save the current values of all of its settings.
        const FlagField& saveCurrentSettings() const { return save_current_settings_; }
    
      private:
        HebiInfoPtr const internal_;
    
        Actuator actuator_;
    
        StringField name_;
        StringField family_;
        FlagField save_current_settings_;
    
        HEBI_DISABLE_COPY_MOVE(Settings)
    };

  public:
    #ifndef DOXYGEN_OMIT_INTERNAL
    /**
     * C-style info object.
     * NOTE: this should not be used except by internal library functions!
     */
    HebiInfoPtr const internal_;
    #endif // DOXYGEN_OMIT_INTERNAL

  private:
    /**
     * True if this object is responsible for creating and destroying the
     * underlying C pointer; false otherwise.
     */
    bool manage_pointer_lifetime_;

  public:
    /**
     * \brief Create a new info object for a single module.
     */
    Info();
    #ifndef DOXYGEN_OMIT_INTERNAL
    /**
     * \brief Wraps an existing C-style info object; object lifetime is assumed to
     * be managed by the caller.
     *
     * NOTE: this should not be used except by internal library functions!
     */
    Info(HebiInfoPtr info);
    #endif // DOXYGEN_OMIT_INTERNAL
    /**
     * \brief Move constructor (necessary for containment in STL template classes)
     */
    Info(Info&& other);

    /**
     * \brief Destructor cleans up info object as necessary.
     */
    virtual ~Info() noexcept; /* annotating specified destructor as noexcept is best-practice */

    // With all submessage and field getters: Note that the returned reference
    // should not be used after the lifetime of this parent.

    // Submessages -------------------------------------------------------------

    /// Module settings that are typically changed at a slower rate.
    const Settings& settings() const;

    // Subfields -------------------------------------------------------------

    /// The module's LED.
    const LedField& led() const;

  private:
    Settings settings_;

    LedField led_;

    /**
     * Disable copy constructor/assignment operators
     */
    HEBI_DISABLE_COPY(Info)

    /* Disable move assigment operator. */
    Info& operator= (const Info&& other) = delete;
};

} // namespace hebi

#endif // INFO_HPP
