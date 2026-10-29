// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Optional;

/*
 * KinematicMotor: Defines a subsystem that controls a motor using velocity values.
 * Operates in the default unit system specified by the motor controller's velocity
 * conversion factor.
 */

public abstract class KinematicMotor extends SubsystemBase {
  protected RelativeEncoder encoder;

  // Defines positive or negative limits.
  // The motor will not move in the positive or negative direction if the
  // positive or negative limit is active.
  protected Optional<DigitalInput> positiveLimit;
  protected Optional<DigitalInput> negativeLimit;

  // Create a PIDController; Controls how the motor ramps up to the correct speed.
  protected PIDController pid = new PIDController(0.0, 0.0, 0.0);
  // Not active by default; Make sure to activate and deactivate when entering and exiting a teleop
  protected boolean active = false;

  protected double targetVelocity = 0.0;
  protected double outputPower = 0.0;

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

  protected abstract void setVoltage(double v);

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
        outputPower = Math.min(0.0, output); // Prevent being positive
      } else if (negativeLimit.isPresent() && !negativeLimit.get().get()) {
        outputPower = Math.max(0.0, output); // Prevent being negative
      } else {
        outputPower = output;
      }

      // Apply the PIDController's output to move the motor
      setVoltage(outputPower);
    }
  }
}
