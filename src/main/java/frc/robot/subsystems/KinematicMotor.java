// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Optional;

/*
 * KinematicMotor: Defines a subsystem that controls a motor using velocity values.
 * Operates in the default unit system specified by the motor controller's velocity
 * conversion factor.
 */

public class KinematicMotor extends SubsystemBase {
  private final SparkMax motor;
  private final RelativeEncoder encoder;

  private final Optional<DigitalInput> positiveLimit;
  private final Optional<DigitalInput> negativeLimit;

  private final PIDController pid;
  private boolean active;

  private double targetVelocity = 0.0;
  private double outputPower = 0.0;

  public KinematicMotor(SparkMax motor, DigitalInput positiveLimit, DigitalInput negativeLimit) {

    System.out.println(
        "NOTE: Ensure that the motor's velocity conversion factor is set to the "
            + "correct value!");

    this.motor = motor;

    // A relative encoder is fine for velocity calculations
    this.encoder = this.motor.getEncoder();

    // Define positive and negative limits
    // The motor will not turn in the positive direction if the positive limit is active.
    // The motor will not turn in the negative direction if the negative limit is active.
    this.positiveLimit = (positiveLimit != null) ? Optional.of(positiveLimit) : Optional.empty();
    this.negativeLimit = (negativeLimit != null) ? Optional.of(negativeLimit) : Optional.empty();

    // Create a PIDController; Controls how the motor ramps up to the correct speed.
    this.pid = new PIDController(0.0, 0.01, 0.0); // TODO: tune KinematicMotor PID Coefficients
    this.pid.setTolerance(0.2);

    this.active = false;
  }

  public KinematicMotor(SparkMax motor, double maxRPM) {
    this(motor, null, null);
  }

  public double getVelocity() {
    return encoder.getVelocity();
  }

  public double getTargetVelocity() {
    return targetVelocity;
  }

  public double getOutputPower() {
    return outputPower;
  }

  public void setTargetVelocity(double v) {
    targetVelocity = v;
  }

  public void activate() {
    this.active = true;
  }

  public void deactivate() {
    this.active = false;
    this.pid.reset(); // Reset PID
  }

  @Override
  public void periodic() {

    if (active) {
      // Use the PIDController
      double output = pid.calculate(encoder.getVelocity(), targetVelocity);

      // Clamp output of PIDController based on which limits are active
      if (positiveLimit.isPresent() && !positiveLimit.get().get()) {
        outputPower = Math.max(Math.min(0.0, output), -1.0); // Prevent being positive
      } else if (negativeLimit.isPresent() && !negativeLimit.get().get()) {
        outputPower = Math.min(Math.max(0.0, output), 1.0); // Prevent being negative
      } else {
        outputPower = Math.min(Math.max(-1.0, output), 1.0); // Clamp within range of -1.0 to 1.0
      }

      // Apply the PIDController's output to move the motor
      motor.setVoltage(outputPower);
    }
  }
}
