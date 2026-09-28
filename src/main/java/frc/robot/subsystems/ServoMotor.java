// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * ServoMotor: Operates a SparkMax motor using position instead of velocity or power
 */
public class ServoMotor extends SubsystemBase {
  private final SparkMax motor;
  private final SparkAbsoluteEncoder encoder;

  private final Optional<DigitalInput> positiveLimit;
  private final Optional<DigitalInput> negativeLimit;

  private final PIDController pid;

  private double targetPosition;
  private double maxPower = 0.1;

  /** Creates a new ServoMotor. */
  public ServoMotor(SparkMax motor, double startingPosition) {
    this.motor = motor;
    // We want to use absolute encoder so that we don't have to manually set its
    // initial position every time we run the robot
    this.encoder = this.motor.getAbsoluteEncoder();
    this.positiveLimit = Optional.empty();
    this.negativeLimit = Optional.empty();

    this.pid = new PIDController(0.12, 0.0, 0.01);

    this.targetPosition = startingPosition;
  }

  public ServoMotor(SparkMax motor, double startingPosition,
    DigitalInput positiveLimit, DigitalInput negativeLimit) {
    this.motor = motor;
    // We want to use absolute encoder so that we don't have to manually set its
    // initial position every time we run the robot
    this.encoder = this.motor.getAbsoluteEncoder();
    this.positiveLimit = (positiveLimit != null) ? Optional.of(positiveLimit) : Optional.empty();
    this.negativeLimit = (negativeLimit != null) ? Optional.of(negativeLimit) : Optional.empty();
    
    this.pid = new PIDController(0.15, 0.05, 0.01);

    this.targetPosition = startingPosition;
  }

  public double getPosition() {
    return encoder.getPosition();
  }

  public double getTargetPosition() {
    return targetPosition;
  }

  public void setTargetPosition(double targetPosition) {
    this.targetPosition = targetPosition;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // Adjust velocity based on current error
    double targetPower = pid.calculate(encoder.getPosition(), targetPosition);

    System.out.println("PID_CALC: " + targetPower);

    // Prevent motor from going past a set limit
    if (positiveLimit.isPresent() && !positiveLimit.get().get()) {
      targetPower = Math.max(Math.min(0.0, targetPower), maxPower);
    } else if (negativeLimit.isPresent() && !negativeLimit.get().get()) {
      targetPower = Math.min(Math.max(0.0, targetPower), -maxPower);
    } else {
      targetPower = Math.min(Math.max(-maxPower, targetPower), maxPower);
    }

    System.out.println("CP: " + getPosition() + " TP: " + targetPosition + " TV: " + targetPower);

    motor.set(targetPower);
  }
}
