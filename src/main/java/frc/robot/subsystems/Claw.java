// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Claw extends SubsystemBase {
  protected Compressor claw_compressor;

  protected DoubleSolenoid claw_solenoid;
  protected boolean claw_solenoid_active = false;
  protected boolean claw_closed;

  protected ServoMotor claw_motor;
  protected boolean claw_motor_active = false;

  /** Creates a new Claw. */
  public Claw(
      int sparkMaxCANID,
      PneumaticsModuleType pneumaticsModuleType,
      int forwardChannel,
      int reverseChannel) {
    this.claw_compressor = new Compressor(pneumaticsModuleType);
    this.claw_solenoid = new DoubleSolenoid(pneumaticsModuleType, forwardChannel, reverseChannel);
    this.claw_motor =
        new SparkMaxServoMotor(
            new SparkMax(sparkMaxCANID, MotorType.kBrushless),
            0.68,
            false,
            null,
            null,
            0.05,
            0.05,
            0.0);
    this.claw_closed = false;
  }

  public void activate(boolean motor, boolean solenoid) {
    claw_motor_active = motor;
    if (motor) {
      this.claw_motor.activate();
    } else {
      this.claw_motor.deactivate();
    }

    claw_solenoid_active = solenoid;
    if (solenoid) {
      this.claw_compressor.enableDigital();
    } else {
      this.claw_compressor.disable();
    }
  }

  public void activate() {
    activate(true, true);
  }

  public void deactivate() {
    activate(false, false);
  }

  public void dropClaw() {
    this.claw_motor.setTargetPosition(0.111);
  }

  public void liftClaw() {
    this.claw_motor.setTargetPosition(0.68);
  }

  public void openClaw() {
    claw_closed = false;
  }

  public void closeClaw() {
    claw_closed = true;
  }

  public double debug_getEncoderOutput() {
    return this.claw_motor.getEncoderPosition();
  }

  public double debug_getMotorOutput() {
    return this.claw_motor.getOutputPower();
  }

  @Override
  public void periodic() {
    if (claw_motor_active) {
      // Add correction value to claw motor output

      // Recording some data here:
      // Arm Length = 0.3m
      // Claw Weight = ~2 lbs
      // Torque per volt = 0.217 N * m/v

      // Given this, we can determine that, in order
      // to keep the motor in equilibrium:
      // V = 18.8 * cos(theta)
      // where theta is the angle above the horizontal.

      claw_motor.periodic();
    }

    if (claw_solenoid_active) {
      if (claw_closed) {
        claw_solenoid.set(DoubleSolenoid.Value.kForward);
      } else {
        claw_solenoid.set(DoubleSolenoid.Value.kReverse);
      }
    } else {
      claw_solenoid.set(DoubleSolenoid.Value.kOff);
    }
  }
}
