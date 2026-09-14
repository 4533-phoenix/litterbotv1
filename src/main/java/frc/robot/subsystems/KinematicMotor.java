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
  private final double adjustIncrement = 0.05;

  private final SparkMax motor;
  private final RelativeEncoder encoder;
  private final double wheelRadius; // In Meters
  private final double wheelCircumference; // In Meters

  // Note: Using SI units (Meters, Seconds)
  private double velocityCurrent = 0.0;
  private double timeCurrent = 0.0;
  
  private double velocityPast = 0.0;
  private double timePast = 0.0;

  private double targetVelocity = 0.0;
  private double targetAcceleration = 0.0;


  // Adjusts itself depending on current motor speed to match the targetVelocity
  private double outputVelocity = 0.0;

  public KinematicMotor(SparkMax motor, double wheelRadius) {
    this.motor = motor;
    this.encoder = this.motor.getEncoder();
    this.wheelRadius = wheelRadius;
    this.wheelCircumference = (2 * Math.PI * wheelRadius);
  }

  // In S
  private double getTimeDelta() {
    return (timeCurrent - timePast);
  }

  // In M
  public double getDistance() {
    return encoder.getPosition() * wheelCircumference;
  }

  // In M/S
  public double getVelocity() {
    return encoder.getVelocity() * wheelCircumference * (1.0 / 60.0);
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

    velocityCurrent = encoder.getVelocity() * wheelCircumference; // In Meters
    timeCurrent = RobotController.getTime() / 1000000.0; // In Seconds

    // Update variables, set motor speed
    targetVelocity += targetAcceleration * getTimeDelta();
    
    // Automatically adjust outputVelocity to match targetVelocity
    if (getVelocity() != targetVelocity && targetVelocity != 0.0) { // Make sure we don't divide by 0
      // Dividing by targetVelocity puts it in a -1.0 to 1.0 range
      outputVelocity += (targetVelocity - getVelocity()) / targetVelocity;
      //outputVelocity = Math.max(-1.0, Math.min(1.0, outputVelocity)); // Clamp
    } else if (targetVelocity == 0.0) {
      outputVelocity = 0;
    }
    
    motor.set(outputVelocity);

    System.out.println(
      "TV: " + targetVelocity + " " +
      "TA: " + targetAcceleration + " " +
      "CV: " + getVelocity() + " " +
      "OV: " + outputVelocity
    );
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
