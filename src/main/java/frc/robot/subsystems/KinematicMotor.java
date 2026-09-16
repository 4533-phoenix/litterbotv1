// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.RobotController;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;

public class KinematicMotor extends SubsystemBase {
  private final SparkMax motor;
  private final RelativeEncoder encoder;
  private final PIDController pid;

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
    this.wheelRadius = wheelRadius;
    this.wheelCircumference = (2 * Math.PI * wheelRadius);
    this.pid = new PIDController(0.15, 0.0, 0.01);
    this.gearRatio = gearRatio; // Represented as to / from
  }

  // In S
  private double getTimeDelta() {
    return (timeCurrent - timePast);
  }

  // In M
  public double getDistance() {
    return encoder.getPosition() * wheelCircumference * gearRatio;
  }

  // In M/S
  public double getVelocity() {
    return (encoder.getVelocity() * wheelCircumference * gearRatio) / 60.0;
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

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
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
    outputVelocity = pid.calculate(velocityCurrent, targetVelocity);

    outputVelocity = Math.max(-1.0, Math.min(1.0, outputVelocity));

    System.out.println("PID: " + outputVelocity);
    System.out.println("TV: " + targetVelocity);
    System.out.println("CV: " + getVelocity());

    motor.set(outputVelocity);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
