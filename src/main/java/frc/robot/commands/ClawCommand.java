// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Claw;
import java.util.function.Supplier;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ClawCommand extends Command {
  private final Claw claw;
  private final Supplier<Boolean> closeClaw;
  private final Supplier<Boolean> dropClaw;

  public ClawCommand(Claw c, Supplier<Boolean> closeClaw, Supplier<Boolean> dropClaw) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.claw = c;
    this.closeClaw = closeClaw;
    this.dropClaw = dropClaw;
    addRequirements(this.claw);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  @Override
  public void execute() {
    if (this.closeClaw.get()) {
      claw.closeClaw();
    } else {
      claw.openClaw();
    }

    if (this.dropClaw.get()) {
      claw.dropClaw();
    } else {
      claw.liftClaw();
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
