// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
  private final SimpleMotorFeedforward ff;

  private double targetPosition;
  private double currentPosition;
  private double currentSensorPosition;
  private int loops = 0; // If the motor wraps around over 1.0, increment. If over 0.0, decrement
  private double maxPower = 0.05;

  /** Creates a new ServoMotor. */
  public ServoMotor(SparkMax motor, double startingPosition,
    DigitalInput positiveLimit, DigitalInput negativeLimit) {
    this.motor = motor;
    // We want to use absolute encoder so that we don't have to manually set its
    // initial position every time we run the robot
    this.encoder = this.motor.getAbsoluteEncoder();
    this.currentSensorPosition = this.encoder.getPosition();
    this.currentPosition = this.currentSensorPosition;

    this.positiveLimit = (positiveLimit != null) ? Optional.of(positiveLimit) : Optional.empty();
    this.negativeLimit = (negativeLimit != null) ? Optional.of(negativeLimit) : Optional.empty();
    
    this.pid = new PIDController(0.15, 0.0, 0.01);
    this.pid.setTolerance(0.05);

    this.ff = new SimpleMotorFeedforward(10, 20);

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

  public void setTargetPosition(double targetPosition) {
    this.targetPosition = targetPosition;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // Adjust velocity based on current error
    double targetPower = 
      -pid.calculate(currentPosition, targetPosition)
    //  + ff.calculate(encoder.getVelocity())
      ;

    //System.out.println("TARGET: " + targetPosition + " PID_CALC: " + targetPower + " POS: " + getPosition());
    SmartDashboard.putNumber("targetPosition", targetPosition);
    SmartDashboard.putNumber("currentPosition", getPosition());
    SmartDashboard.putNumber("targetPower", targetPower);

    // Prevent motor from going past a set limit
    if (positiveLimit.isPresent() && !positiveLimit.get().get()) {
      targetPower = Math.max(Math.min(0.0, targetPower), -maxPower);
    } else if (negativeLimit.isPresent() && !negativeLimit.get().get()) {
      targetPower = Math.min(Math.max(0.0, targetPower), maxPower);
    } else {
      targetPower = Math.min(Math.max(-maxPower, targetPower), maxPower);
    }

    motor.set(targetPower);

    // If the velocity is negative but the change in position is positive (AKA if it wrapped around 0.0)
    if ((this.encoder.getVelocity() < 0.0) && (encoder.getPosition() - currentSensorPosition > 0.5) ) {
      loops--; // decrement loops
    } else if ((this.encoder.getVelocity() > 0.0) && (encoder.getPosition() - currentSensorPosition < -0.5)) {
      // If the velocity is positive but the change in position is negative (AKA if it wrapped around 1.0)
      loops++; // increment loops
    }

    currentSensorPosition = encoder.getPosition();
    currentPosition = currentSensorPosition + loops;
  }
}
