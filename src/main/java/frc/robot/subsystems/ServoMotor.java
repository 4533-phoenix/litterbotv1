// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

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
public class ServoMotor extends SubsystemBase {
  private final SparkMax motor;
  private final SparkAbsoluteEncoder encoder;

  private final Optional<DigitalInput> positiveLimit;
  private final Optional<DigitalInput> negativeLimit;

  private final PIDController pid;
  private boolean active;

  private double targetPosition;
  private double outputPower = 0.0;

  private double currentPosition;
  private double currentSensorPosition;
  private int loops = 0; // If the motor wraps around over 1.0, increment. If over 0.0, decrement

  /** Creates a new ServoMotor. */
  public ServoMotor(
      SparkMax motor,
      double startingPosition,
      DigitalInput positiveLimit,
      DigitalInput negativeLimit) {

    System.out.println(
        "NOTE: Ensure that the motor's positional conversion factor is set to the "
            + "correct value!");

    this.motor = motor;
    // We want to use absolute encoder so that we don't have to manually set its
    // initial position every time we run the robot
    this.encoder = this.motor.getAbsoluteEncoder();
    this.currentSensorPosition = this.encoder.getPosition();
    this.currentPosition = this.currentSensorPosition;
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

  public ServoMotor(SparkMax motor, double startingPosition) {
    this(motor, startingPosition, null, null);
  }

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

  @Override
  public void periodic() {
    // Since an absolute encoder only measures in a range of 0.0 to 1.0, we can't get multiple rotations
    // from a direct sensor output. We have to do extra stuff in order to account for multiple rotations.
    // Doing this prevents overshooting behavior.

    // If the velocity is negative but the change in position is positive (AKA if it wrapped around
    // 0.0)
    if ((this.encoder.getVelocity() < 0.0) && (encoder.getPosition() - currentSensorPosition > 0.5)) {
      loops--; // decrement loops
    } else if ((this.encoder.getVelocity() > 0.0) && (encoder.getPosition() - currentSensorPosition < -0.5)) {
      // If the velocity is positive but the change in position is negative (AKA if it wrapped
      // around 1.0)
      loops++; // increment loops
    }

    // Update measured and calculated position
    currentSensorPosition = encoder.getPosition();
    currentPosition = currentSensorPosition + loops;

    if (active) {
      // Adjust velocity based on current error
      double output = -pid.calculate(currentPosition, targetPosition);

      // Prevent motor from going past a set limit
      if (positiveLimit.isPresent() && !positiveLimit.get().get()) {
        outputPower = Math.max(Math.min(0.0, output), -1.0);
      } else if (negativeLimit.isPresent() && !negativeLimit.get().get()) {
        outputPower = Math.min(Math.max(0.0, output), 1.0);
      } else {
        outputPower = Math.min(Math.max(-1.0, output), 1.0);
      }

      // Apply calculated motor power
      motor.set(outputPower);
    }
  }    
}
