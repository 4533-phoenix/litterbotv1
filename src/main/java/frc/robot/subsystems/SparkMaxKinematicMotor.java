// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import java.util.Optional;

public class SparkMaxKinematicMotor extends KinematicMotor {

  private SparkMax motor;

  /** Creates a new SparkMaxKinematicMotor. */
  public SparkMaxKinematicMotor(
      SparkMax motor, DigitalInput positiveLimit, DigitalInput negativeLimit) {
    this.motor = motor;
    this.encoder = this.motor.getEncoder();

    // Define positive and negative limits
    // The motor will not turn in the positive direction if the positive limit is active.
    // The motor will not turn in the negative direction if the negative limit is active.
    this.positiveLimit = (positiveLimit != null) ? Optional.of(positiveLimit) : Optional.empty();
    this.negativeLimit = (negativeLimit != null) ? Optional.of(negativeLimit) : Optional.empty();

    this.pid.setTolerance(0.2);
  }

  public SparkMaxKinematicMotor(SparkMax motor) {
    this(motor, null, null);
  }

  @Override
  protected void setVoltage(double v) {
    motor.setVoltage(v);
  }
}
