#ifndef FEEDBACK_HPP
#define FEEDBACK_HPP

#include "hebi_feedback.h"
#include "color.hpp"
#include "vector_3_f.hpp"
#include "util.hpp"

namespace hebi {

/// \brief Feedback objects have various fields representing feedback from modules;
/// which fields are populated depends on the module type and various other settings.
/// 
/// This object has a hierarchical structure -- there are some direct general-purpose
/// fields at the top level, and many more specific fields contained in different
/// nested subobjects.
/// 
/// The subobjects contain references to the parent feedback object, and so should
/// not be used after the parent object has been destroyed.
/// 
/// The fields in the feedback object are typed; generally, these are optional-style
/// read-only fields (i.e., have the concept of has/get), although the return types
/// and exact interface vary slightly between fields. Where appropriate, the explicit
/// bool operator has been overridden so that you can shortcut @c if(field.has()) by
/// calling @c if(field).
/// 
/// Although this header file can be used to look at the hierarchy of the messages,
/// in general the online documentation at apidocs.hebi.us presents this information.
/// in a more readable form.
class Feedback final
{
  // Note: this is 'protected' instead of 'private' for easier use with Doxygen
  protected:
    /// \brief A message field representable by a single-precision floating point value.
    class FloatField final
    {
      public:
        #ifndef DOXYGEN_OMIT_INTERNAL
        FloatField(HebiFeedbackPtr internal, FeedbackFloatField field);
        #endif // DOXYGEN_OMIT_INTERNAL
        /// \brief Allows casting to a bool to check if the field has a value
        /// without directly calling @c has().
        ///
        /// This can be used as in the following (assuming 'parent' is a parent message,
        /// and this field is called 'myField')
        /// \code{.cpp}
        /// Feedback::FloatField& f = parent.myField();
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
        HebiFeedbackPtr const internal_;
        FeedbackFloatField const field_;

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
        HighResAngleField(HebiFeedbackPtr internal, FeedbackHighResAngleField field);
        #endif // DOXYGEN_OMIT_INTERNAL
        /// \brief Allows casting to a bool to check if the field has a value
        /// without directly calling @c has().
        ///
        /// This can be used as in the following (assuming 'parent' is a parent message,
        /// and this field is called 'myField')
        /// \code{.cpp}
        /// Feedback::HighResAngleField& f = parent.myField();
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

      private:
        HebiFeedbackPtr const internal_;
        FeedbackHighResAngleField const field_;

        HEBI_DISABLE_COPY_MOVE(HighResAngleField)
    };

    /// \brief A message field containing a numbered set of single-precision
    /// floating point values.
    class NumberedFloatField final
    {
      public:
        #ifndef DOXYGEN_OMIT_INTERNAL
        NumberedFloatField(HebiFeedbackPtr internal, FeedbackNumberedFloatField field);
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

      private:
        HebiFeedbackPtr const internal_;
        FeedbackNumberedFloatField const field_;

        HEBI_DISABLE_COPY_MOVE(NumberedFloatField)
    };

    /// \brief A message field representable by a 3-D vector of single-precision
    /// floating point values.
    class Vector3fField final
    {
      public:
        #ifndef DOXYGEN_OMIT_INTERNAL
        Vector3fField(HebiFeedbackPtr internal, FeedbackVector3fField field);
        #endif // DOXYGEN_OMIT_INTERNAL
        /// \brief Allows casting to a bool to check if the field has a value
        /// without directly calling @c has().
        ///
        /// This can be used as in the following (assuming 'parent' is a parent message,
        /// and this field is called 'myField')
        /// \code{.cpp}
        /// Feedback::Vector3fField& f = parent.myField();
        /// if (f)
        ///   std::cout << "Field has value!" << std::endl;
        /// else
        ///   std::cout << "Field has no value!" << std::endl;
        /// \endcode
        explicit operator bool() const;
        /// \brief True if (and only if) the field has a value.
        bool has() const;
        /// \brief If the field has a value, returns that value; otherwise,
        /// returns a default.
        Vector3f get() const;

      private:
        HebiFeedbackPtr const internal_;
        FeedbackVector3fField const field_;

        HEBI_DISABLE_COPY_MOVE(Vector3fField)
    };

    /// \brief A message field for interfacing with an LED.
    class LedField final
    {
      public:
        #ifndef DOXYGEN_OMIT_INTERNAL
        LedField(HebiFeedbackPtr internal, FeedbackLedField field);
        #endif // DOXYGEN_OMIT_INTERNAL
        /// \brief Allows casting to a bool to check if the LED color is set
        /// without directly calling @c hasColor().
        ///
        /// This can be used as in the following (assuming 'parent' is a parent message,
        /// and this field is called 'myField')
        /// \code{.cpp}
        /// Feedback::LedField& f = parent.myField();
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
        HebiFeedbackPtr const internal_;
        FeedbackLedField const field_;

        HEBI_DISABLE_COPY_MOVE(LedField)
    };

    /// Actuator-specific feedback.
    class Actuator final
    {
      public:
        #ifndef DOXYGEN_OMIT_INTERNAL
        Actuator(HebiFeedbackPtr internal)
          : internal_(internal),
            velocity_(internal, FeedbackFloatVelocity),
            torque_(internal, FeedbackFloatTorque),
            velocity_command_(internal, FeedbackFloatVelocityCommand),
            torque_command_(internal, FeedbackFloatTorqueCommand),
            deflection_(internal, FeedbackFloatDeflection),
            deflection_velocity_(internal, FeedbackFloatDeflectionVelocity),
            motor_velocity_(internal, FeedbackFloatMotorVelocity),
            motor_current_(internal, FeedbackFloatMotorCurrent),
            motor_sensor_temperature_(internal, FeedbackFloatMotorSensorTemperature),
            motor_winding_current_(internal, FeedbackFloatMotorWindingCurrent),
            motor_winding_temperature_(internal, FeedbackFloatMotorWindingTemperature),
            motor_housing_temperature_(internal, FeedbackFloatMotorHousingTemperature),
            position_(internal, FeedbackHighResAnglePosition),
            position_command_(internal, FeedbackHighResAnglePositionCommand)
        {
        }
        #endif // DOXYGEN_OMIT_INTERNAL
    
        // With all submessage and field getters: Note that the returned reference
        // should not be used after the lifetime of this parent.
    
        // Subfields ----------------
    
        /// Velocity of the module output (post-spring), in radians/second.
        const FloatField& velocity() const { return velocity_; }
        /// Torque at the module output, in N * m.
        const FloatField& torque() const { return torque_; }
        /// Commanded velocity of the module output (post-spring), in radians/second.
        const FloatField& velocityCommand() const { return velocity_command_; }
        /// Commanded torque at the module output, in N * m.
        const FloatField& torqueCommand() const { return torque_command_; }
        /// Difference (in radians) between the pre-spring and post-spring output position.
        const FloatField& deflection() const { return deflection_; }
        /// Velocity (in radians/second) of the difference between the pre-spring and post-spring output position.
        const FloatField& deflectionVelocity() const { return deflection_velocity_; }
        /// The velocity (in radians/second) of the motor shaft.
        const FloatField& motorVelocity() const { return motor_velocity_; }
        /// Current supplied to the motor.
        const FloatField& motorCurrent() const { return motor_current_; }
        /// The temperature from a sensor near the motor housing.
        const FloatField& motorSensorTemperature() const { return motor_sensor_temperature_; }
        /// The estimated current in the motor windings.
        const FloatField& motorWindingCurrent() const { return motor_winding_current_; }
        /// The estimated temperature of the motor windings.
        const FloatField& motorWindingTemperature() const { return motor_winding_temperature_; }
        /// The estimated temperature of the motor housing.
        const FloatField& motorHousingTemperature() const { return motor_housing_temperature_; }
        /// Position of the module output (post-spring), in radians.
        const HighResAngleField& position() const { return position_; }
        /// Commanded position of the module output (post-spring), in radians.
        const HighResAngleField& positionCommand() const { return position_command_; }
    
      private:
        HebiFeedbackPtr const internal_;
    
        FloatField velocity_;
        FloatField torque_;
        FloatField velocity_command_;
        FloatField torque_command_;
        FloatField deflection_;
        FloatField deflection_velocity_;
        FloatField motor_velocity_;
        FloatField motor_current_;
        FloatField motor_sensor_temperature_;
        FloatField motor_winding_current_;
        FloatField motor_winding_temperature_;
        FloatField motor_housing_temperature_;
        HighResAngleField position_;
        HighResAngleField position_command_;
    
        HEBI_DISABLE_COPY_MOVE(Actuator)
    };

    /// Inertial measurement unit feedback (accelerometers and gyros).
    class Imu final
    {
      public:
        #ifndef DOXYGEN_OMIT_INTERNAL
        Imu(HebiFeedbackPtr internal)
          : internal_(internal),
            accelerometer_(internal, FeedbackVector3fAccelerometer),
            gyro_(internal, FeedbackVector3fGyro)
        {
        }
        #endif // DOXYGEN_OMIT_INTERNAL
    
        // With all submessage and field getters: Note that the returned reference
        // should not be used after the lifetime of this parent.
    
        // Subfields ----------------
    
        /// Accelerometer data, in m/s^2.
        const Vector3fField& accelerometer() const { return accelerometer_; }
        /// Gyro data, in radians/second.
        const Vector3fField& gyro() const { return gyro_; }
    
      private:
        HebiFeedbackPtr const internal_;
    
        Vector3fField accelerometer_;
        Vector3fField gyro_;
    
        HEBI_DISABLE_COPY_MOVE(Imu)
    };

  public:
    #ifndef DOXYGEN_OMIT_INTERNAL
    /**
     * C-style feedback object.
     * NOTE: this should not be used except by internal library functions!
     */
    HebiFeedbackPtr const internal_;
    #endif // DOXYGEN_OMIT_INTERNAL

  private:
    /**
     * True if this object is responsible for creating and destroying the
     * underlying C pointer; false otherwise.
     */
    bool manage_pointer_lifetime_;

  public:
    /**
     * \brief Create a new feedback object for a single module.
     */
    Feedback();
    #ifndef DOXYGEN_OMIT_INTERNAL
    /**
     * \brief Wraps an existing C-style feedback object; object lifetime is assumed to
     * be managed by the caller.
     *
     * NOTE: this should not be used except by internal library functions!
     */
    Feedback(HebiFeedbackPtr feedback);
    #endif // DOXYGEN_OMIT_INTERNAL
    /**
     * \brief Move constructor (necessary for containment in STL template classes)
     */
    Feedback(Feedback&& other);

    /**
     * \brief Destructor cleans up feedback object as necessary.
     */
    virtual ~Feedback() noexcept; /* annotating specified destructor as noexcept is best-practice */

    // With all submessage and field getters: Note that the returned reference
    // should not be used after the lifetime of this parent.

    // Submessages -------------------------------------------------------------

    /// Actuator-specific feedback.
    const Actuator& actuator() const;
    /// Inertial measurement unit feedback (accelerometers and gyros).
    const Imu& imu() const;

    // Subfields -------------------------------------------------------------

    /// Ambient temperature inside the module (measured at the IMU chip), in degrees Celsius.
    const FloatField& boardTemperature() const;
    /// Temperature of the processor chip, in degrees Celsius.
    const FloatField& processorTemperature() const;
    /// Bus voltage that the module is running at (in Volts).
    const FloatField& voltage() const;
    #ifndef DOXYGEN_OMIT_INTERNAL
    /// Values for internal debug functions (channel 1-9 available).
    const NumberedFloatField& debug() const;
    #endif // DOXYGEN_OMIT_INTERNAL
    /// The module's LED.
    const LedField& led() const;

  private:
    Actuator actuator_;
    Imu imu_;

    FloatField board_temperature_;
    FloatField processor_temperature_;
    FloatField voltage_;
    NumberedFloatField debug_;
    LedField led_;

    /**
     * Disable copy constructor/assignment operators
     */
    HEBI_DISABLE_COPY(Feedback)

    /* Disable move assigment operator. */
    Feedback& operator= (const Feedback&& other) = delete;
};

} // namespace hebi

#endif // FEEDBACK_HPP
