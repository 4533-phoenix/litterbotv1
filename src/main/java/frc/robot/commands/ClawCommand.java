// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Claw;
import java.util.Optional;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ClawCommand extends InstantCommand {
  private final Claw claw;
  private final Optional<Boolean> closeClaw;
  private final Optional<Boolean> dropClaw;

  public ClawCommand(Claw c, Boolean closeClaw, Boolean dropClaw) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.claw = c;
    this.closeClaw = Optional.ofNullable(closeClaw);
    this.dropClaw = Optional.ofNullable(dropClaw);
    addRequirements(this.claw);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (this.closeClaw.isPresent()) {
      if (this.closeClaw.get()) {
        claw.closeClaw();
      } else {
        claw.openClaw();
      }
    }

    if (this.dropClaw.isPresent()) {
      if (this.dropClaw.get()) {
        claw.dropClaw();
      } else {
        claw.liftClaw();
      }
    }
  }
}
