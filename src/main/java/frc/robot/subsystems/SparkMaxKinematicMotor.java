// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import java.util.Optional;

public class SparkMaxKinematicMotor extends KinematicMotor {

  private SparkMax motor;

  /** Creates a new SparkMaxKinematicMotor. */
  public SparkMaxKinematicMotor(
      SparkMax motor,
      DigitalInput positiveLimit,
      DigitalInput negativeLimit,
      double kP,
      double kI,
      double kD) {
    this.motor = motor;
    this.encoder = this.motor.getEncoder();

    this.pid = new PIDController(kP, kI, kD);
    this.pid.setTolerance(0.2);

    // Define positive and negative limits
    // The motor will not turn in the positive direction if the positive limit is active.
    // The motor will not turn in the negative direction if the negative limit is active.
    this.positiveLimit = Optional.ofNullable(positiveLimit);
    this.negativeLimit = Optional.ofNullable(negativeLimit);
  }

  public SparkMaxKinematicMotor(
      SparkMax motor, DigitalInput positiveLimit, DigitalInput negativeLimit) {
    this(motor, positiveLimit, negativeLimit, 0.02, 0.3, 0.0);
  }

  public SparkMaxKinematicMotor(SparkMax motor) {
    this(motor, null, null);
  }

  @Override
  protected void setVoltage(double v) {
    motor.setVoltage(v);
  }
}
