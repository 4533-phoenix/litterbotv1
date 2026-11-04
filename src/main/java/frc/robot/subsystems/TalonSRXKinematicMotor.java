// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.SensorCollection;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import java.util.Optional;

public class TalonSRXKinematicMotor extends KinematicMotor {

  private final TalonSRX motor;

  private final SensorCollection encoder;
  private double velocityScalar;

  /** Creates a new TalonSRXKinematicMotor. */
  public TalonSRXKinematicMotor(
      TalonSRX motor,
      double velocityScalar,
      DigitalInput positiveLimit,
      DigitalInput negativeLimit,
      double kP,
      double kI,
      double kD) {
    this.motor = motor;
    this.encoder = this.motor.getSensorCollection();
    this.velocityScalar = velocityScalar;

    this.pid = new PIDController(kP, kI, kD);
    this.pid.setTolerance(0.2);

    // Define positive and negative limits
    // The motor will not turn in the positive direction if the positive limit is active.
    // The motor will not turn in the negative direction if the negative limit is active.
    this.positiveLimit = Optional.ofNullable(positiveLimit);
    this.negativeLimit = Optional.ofNullable(negativeLimit);
  }

  public TalonSRXKinematicMotor(
      TalonSRX motor,
      DigitalInput positiveLimit,
      DigitalInput negativeLimit,
      double kP,
      double kI,
      double kD) {
    this(motor, 10.0 / 4096.0, positiveLimit, negativeLimit, kP, kI, kD);
  }

  public TalonSRXKinematicMotor(
      TalonSRX motor, DigitalInput positiveLimit, DigitalInput negativeLimit) {
    this(motor, positiveLimit, negativeLimit, 0.02, 0.3, 0.0);
  }

  @Override
  protected void setVoltage(double v) {
    this.motor.set(TalonSRXControlMode.PercentOutput, v);
  }

  @Override
  public double getVelocity() {
    // In this case, the default units are encoder ticks / 100 ms. Multiply by 10.0 / 4096.0 to get
    // RPS
    return this.encoder.getQuadratureVelocity() * velocityScalar;
  }
}
