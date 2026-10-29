// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.KinematicMotor;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class KinematicMotorCommand extends InstantCommand {
  private final KinematicMotor km;
  private final double tv;

  public KinematicMotorCommand(KinematicMotor km, double tv) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.km = km;
    this.tv = tv;
    addRequirements(km);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    km.setTargetVelocity(tv);
  }
}
