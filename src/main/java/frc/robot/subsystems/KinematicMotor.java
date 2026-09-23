// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;

public class KinematicMotor extends SubsystemBase {
  private final SparkMax motor;
  private final RelativeEncoder encoder;

  private final Optional<DigitalInput> positiveLimit;
  private final Optional<DigitalInput> negativeLimit;

  private final PIDController pid;
  private final SimpleMotorFeedforward ff;

  private final double wheelRadius; // In Meters
  private final double wheelCircumference; // In Meters
  private final double gearRatio;

  // Note: Using SI units (Meters, Seconds)
  private double velocityCurrent = 0.0;
  private double timeCurrent = 0.0;
  
  private double velocityPast = 0.0;
  private double timePast = 0.0;

  private double targetVelocity = 0.0;
  private double targetAcceleration = 0.0;


  // Adjusts itself depending on current motor speed to match the targetVelocity
  private double outputVelocity = 0.0;

  public KinematicMotor(SparkMax motor, double wheelRadius, double gearRatio) {
    this.motor = motor;
    this.encoder = this.motor.getEncoder();
    this.positiveLimit = Optional.empty();
    this.negativeLimit = Optional.empty();
    this.wheelRadius = wheelRadius;
    this.wheelCircumference = (2 * Math.PI * wheelRadius);
    this.pid = new PIDController(0.15, 0.0, 0.01);
    this.ff = new SimpleMotorFeedforward(1.0, 0.5);
    this.gearRatio = gearRatio; // Represented as to / from
  }

  public KinematicMotor(SparkMax motor, double wheelRadius, double gearRatio,
    DigitalInput positiveLimit, DigitalInput negativeLimit) {
    this.motor = motor;
    this.encoder = this.motor.getEncoder();
    this.positiveLimit = (positiveLimit != null) ? Optional.of(positiveLimit) : Optional.empty();
    this.negativeLimit = (negativeLimit != null) ? Optional.of(negativeLimit) : Optional.empty();
    this.wheelRadius = wheelRadius;
    this.wheelCircumference = (2 * Math.PI * wheelRadius);
    this.pid = new PIDController(0.15, 0.0, 0.01);
    this.ff = new SimpleMotorFeedforward(1.0, 0.5);
    this.gearRatio = gearRatio; // Represented as to / from
  }

  // In S
  private double getTimeDelta() {
    return (timeCurrent - timePast);
  }

  // In M
  public double getDistance() {
    return encoder.getPosition() * gearRatio * wheelCircumference;
  }

  // In M/S
  public double getVelocity() {
    return (encoder.getVelocity() * gearRatio * wheelCircumference) / 60.0;
  }

  // In M/S^2
  public double getAcceleration() {
    return (velocityCurrent - velocityPast) / (timeCurrent - timePast);
  }

  public void setTargetVelocity(double v) {
    targetVelocity = v;
  }

  public void setTargetAcceleration(double a) {
    targetAcceleration = a;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // Get data from encoders and sensors
    velocityPast = velocityCurrent;
    timePast = timeCurrent;

    velocityCurrent = getVelocity(); // In Meters / Second
    timeCurrent = RobotController.getTime() / 1000000.0; // In Seconds

    // Update variables, set target motor speed
    targetVelocity += targetAcceleration * getTimeDelta();
    
    // Set output velocity
    outputVelocity = pid.calculate(velocityCurrent, targetVelocity); // + ff.calculate(targetVelocity);

    outputVelocity = Math.max(-1.0, Math.min(1.0, outputVelocity));

    System.out.println("OV: " + outputVelocity);

    if (positiveLimit.isPresent()) {
      // If we hit our positive limit
      if (!positiveLimit.get().get()) {
        motor.set(Math.min(0.0, outputVelocity)); // Only allow negative velocities
      } else {
        motor.set(outputVelocity);
      }
    } else if (negativeLimit.isPresent()) {
      // If we hit our negative limit
      if (!negativeLimit.get().get()) {
        motor.set(Math.max(0.0, outputVelocity)); // Only allow positive velocities
      } else {
        motor.set(outputVelocity);
      }
    } else {
      motor.set(outputVelocity);
    }
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
