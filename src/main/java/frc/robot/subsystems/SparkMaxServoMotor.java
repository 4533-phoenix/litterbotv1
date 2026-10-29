// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import java.util.Optional;

public class SparkMaxServoMotor extends ServoMotor {
  private SparkMax motor;
  private Optional<SparkAbsoluteEncoder> encoder;

  /** Creates a new SparkMaxServoMotor. */
  public SparkMaxServoMotor(
      SparkMax motor,
      double startingPosition,
      boolean usingRelativeEncoder,
      DigitalInput positiveLimit,
      DigitalInput negativeLimit,
      double kP,
      double kI,
      double kD) {

    this.motor = motor;

    this.pid = new PIDController(kP, kI, kD);
    this.pid.setTolerance(0.05);

    this.encoder =
        (usingRelativeEncoder) ? Optional.empty() : Optional.of(this.motor.getAbsoluteEncoder());

    this.relativeEncoder =
        (usingRelativeEncoder) ? Optional.of(this.motor.getEncoder()) : Optional.empty();

    this.positiveLimit = Optional.ofNullable(positiveLimit);
    this.negativeLimit = Optional.ofNullable(negativeLimit);
  }

  public SparkMaxServoMotor(
      SparkMax motor,
      double startingPosition,
      boolean usingRelativeEncoder,
      DigitalInput positiveLimit,
      DigitalInput negativeLimit) {
    this(
        motor,
        startingPosition,
        usingRelativeEncoder,
        positiveLimit,
        negativeLimit,
        0.5,
        0.2,
        0.01);
  }

  public SparkMaxServoMotor(SparkMax motor, double startingPosition, boolean usingRelativeEncoder) {
    this(motor, startingPosition, usingRelativeEncoder, null, null);
  }

  public SparkMaxServoMotor(SparkMax motor, double startingPosition) {
    this(motor, startingPosition, true, null, null);
  }

  @Override
  protected boolean absoluteEncoderIsPresent() {
    return this.encoder.isPresent();
  }

  @Override
  protected boolean relativeEncoderIsPresent() {
    return this.relativeEncoder.isPresent();
  }

  @Override
  protected double getEncoderPosition() {
    if (absoluteEncoderIsPresent()) {
      return this.encoder.get().getPosition();
    } else if (relativeEncoderIsPresent()) {
      return this.relativeEncoder.get().getPosition();
    } else {
      return 0.0;
    }
  }

  @Override
  protected double getEncoderVelocity() {
    if (absoluteEncoderIsPresent()) {
      return this.encoder.get().getVelocity();
    } else if (relativeEncoderIsPresent()) {
      return this.relativeEncoder.get().getVelocity();
    } else {
      return 0.0;
    }
  }

  @Override
  protected void setVoltage(double v) {
    motor.setVoltage(v);
  }
}
