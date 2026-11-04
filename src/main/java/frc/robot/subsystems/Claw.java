// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Claw extends SubsystemBase {
  protected boolean active;

  protected DoubleSolenoid claw_solenoid;
  protected boolean claw_active;

  protected ServoMotor claw_motor;

  /** Creates a new Claw. */
  public Claw(
      int sparkMaxCANID,
      PneumaticsModuleType pneumaticsModuleType,
      int forwardChannel,
      int reverseChannel) {
    this.claw_solenoid = new DoubleSolenoid(pneumaticsModuleType, forwardChannel, reverseChannel);
    this.claw_motor =
        new SparkMaxServoMotor(new SparkMax(sparkMaxCANID, MotorType.kBrushless), 0.68, false);
    this.claw_active = false;
  }

  public void activate() {
    this.claw_motor.activate();
    active = true;
  }

  public void deactivate() {
    this.claw_motor.deactivate();
    active = false;
  }

  public void dropClaw() {
    this.claw_motor.setTargetPosition(0.111);
  }

  public void liftClaw() {
    this.claw_motor.setTargetPosition(0.68);
  }

  public void openClaw() {
    claw_active = false;
  }

  public void closeClaw() {
    claw_active = true;
  }

  @Override
  public void periodic() {
    if (active) {
      claw_motor.periodic();
      if (claw_active) {
        claw_solenoid.set(DoubleSolenoid.Value.kForward);
      } else {
        claw_solenoid.set(DoubleSolenoid.Value.kReverse);
      }
    } else {
      claw_solenoid.set(DoubleSolenoid.Value.kOff);
    }
  }
}
