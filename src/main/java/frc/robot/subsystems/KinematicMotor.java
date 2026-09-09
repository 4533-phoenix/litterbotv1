// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.RobotController;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;

public class KinematicMotor extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private final SparkMax motor;
  private final RelativeEncoder encoder;
  private final double wheelRadius;
  private final double wheelCircumference;

  // Note: Using SI units (Meters, Seconds)
  private double velocityCurrent = 0.0;
  private double timeCurrent = 0.0;
  
  private double velocityPast = 0.0;
  private double timePast = 0.0;

  public KinematicMotor(SparkMax motor, double wheelRadius) {
    this.motor = motor;
    this.encoder = this.motor.getEncoder();
    this.wheelRadius = wheelRadius;
    this.wheelCircumference = (2 * Math.PI * wheelRadius);
  }

  // In M
  public double getDistance() {
    return motor.getEncoder().getPosition() * wheelCircumference;
  }

  // In M/S
  public double getVelocity() {
    return motor.getEncoder().getVelocity() * wheelCircumference * (1.0 / 60.0);
  }

  public double getAcceleration() {
    return motor.getEncoder().getVelocity() * wheelCircumference;
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
    velocityPast = velocityCurrent;
    timePast = timeCurrent;

    velocityCurrent = motor.getEncoder()
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
