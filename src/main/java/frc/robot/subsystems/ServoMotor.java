// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Optional;

/**
 * ServoMotor: Defines a subsystem that controls a motor using position values. Operates in the
 * default unit system specified by the motor controller's position conversion factor.
 */
public abstract class ServoMotor extends SubsystemBase {
  //protected final SparkMax motor;
  //protected final Optional<SparkAbsoluteEncoder> encoder;
  protected Optional<RelativeEncoder> relativeEncoder = Optional.empty();

  protected Optional<DigitalInput> positiveLimit = Optional.empty();
  protected Optional<DigitalInput> negativeLimit = Optional.empty();

  protected PIDController pid = new PIDController(0.0, 0.01, 0.0);;
  protected boolean active = false;

  protected double targetPosition = 0.0;
  protected double outputPower = 0.0;

  protected double currentPosition = 0.0;
  protected double currentEncoderPosition = 0.0;
  protected int loops = 0; // If the motor wraps around over 1.0, increment. If over 0.0, decrement

  /** Creates a new ServoMotor. */

  /*
  public ServoMotor(
      SparkMax motor,
      double startingPosition,
      boolean usingRelativeEncoder,
      DigitalInput positiveLimit,
      DigitalInput negativeLimit) {

    System.out.println(
        "NOTE: Ensure that the motor's positional conversion factor is set to the "
            + "correct value!");

    this.motor = motor;
    // Ideally, we want to use an absolute encoder so we don't have to reset its position every time
    // If we are using a relative encoder, startingPosition should be 0
    //this.encoder =
    //    (usingRelativeEncoder) ? Optional.empty() : Optional.of(this.motor.getAbsoluteEncoder());
    //this.relativeEncoder =
    //    (usingRelativeEncoder) ? Optional.of(this.motor.getEncoder()) : Optional.empty();

    // Reset relative encoder to 0 to be safe; We don't know what it was set to before the code was
    // deployed
    if (usingRelativeEncoder) {
      this.relativeEncoder.get().setPosition(0.0);
    }

    this.currentEncoderPosition =
        (usingRelativeEncoder)
            ? this.relativeEncoder.get().getPosition()
            : this.encoder.get().getPosition();

    this.currentPosition = this.currentEncoderPosition;
    // Define positive and negative limits
    // The motor will not turn in the positive direction if the positive limit is active.
    // The motor will not turn in the negative direction if the negative limit is active.
    this.positiveLimit = (positiveLimit != null) ? Optional.of(positiveLimit) : Optional.empty();
    this.negativeLimit = (negativeLimit != null) ? Optional.of(negativeLimit) : Optional.empty();

    // Create a PIDController; Controls how the motor ramps up to the correct speed.
    this.pid = new PIDController(0.0, 0.01, 0.0); // TODO: tune ServoMotor PID Coefficients
    this.pid.setTolerance(0.05);

    this.active = false;

    this.targetPosition = startingPosition;
  }
  */

  public double getPosition() {
    return currentPosition;
  }

  public double getTargetPosition() {
    return targetPosition;
  }

  public double getOutputPower() {
    return outputPower;
  }

  public void setTargetPosition(double targetPosition) {
    this.targetPosition = targetPosition;
  }

  protected abstract boolean absoluteEncoderIsPresent();

  protected abstract boolean relativeEncoderIsPresent();

  protected abstract double getEncoderPosition();

  protected abstract double getEncoderVelocity();

  protected abstract void setVoltage(double v);

  @Override
  public void periodic() {

    // Since an absolute encoder only measures in a range of 0.0 to 1.0, we can't get multiple
    // rotations from a direct sensor output. We have to do extra stuff in order to account for
    // multiple
    // rotations. Doing this prevents overshooting behavior.

    // If the velocity is negative but the change in position is positive (AKA if it wrapped around
    // 0.0)

    if (absoluteEncoderIsPresent()) {
      if ((getEncoderVelocity() < 0.0) && (getEncoderPosition() - currentEncoderPosition > 0.5)) {
        loops--; // decrement loops
      } else if ((getEncoderVelocity() > 0.0)
          && (getEncoderPosition() - currentEncoderPosition < -0.5)) {
        // If the velocity is positive but the change in position is negative (AKA if it wrapped
        // around 1.0)
        loops++; // increment loops
      }
    }

    // Update measured and calculated position
    currentPosition = currentEncoderPosition + loops;
    currentEncoderPosition = getEncoderPosition();

    if (active) {
      // Adjust velocity based on current error
      double output = -pid.calculate(currentPosition, targetPosition);

      // Clamp output of PIDController based on which limits are active
      if (positiveLimit.isPresent() && !positiveLimit.get().get()) {
        outputPower = Math.min(0.0, output); // Prevent being positive
      } else if (negativeLimit.isPresent() && !negativeLimit.get().get()) {
        outputPower = Math.max(0.0, output); // Prevent being negative
      } else {
        outputPower = output;
      }

      // Apply calculated motor power
      setVoltage(outputPower);
    }
  }
}
