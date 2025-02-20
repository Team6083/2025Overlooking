// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstant;

public class ClimberSubsystem extends SubsystemBase {
  /** Creates a new ClimberSubsystem. */
  private final VictorSPX climberMotor;

  public ClimberSubsystem() {
    climberMotor = new VictorSPX(ClimberConstant.kClimberMotorChannel);
    climberMotor.setInverted(ClimberConstant.kClimberMotorInverted);
  }

  public void setMotorSpeed(double speed) {
    climberMotor.set(VictorSPXControlMode.PercentOutput, speed);
  }

  public void stopMotor() {
    climberMotor.set(VictorSPXControlMode.PercentOutput, 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public Command climbUpCmd() {
    Command cmd = runEnd(
        () -> setMotorSpeed(ClimberConstant.kClimbUpSpeed), this::stopMotor);
    cmd.setName("climbUp");
    return cmd;
  }

  public Command ClimbDownCmd() {
    Command cmd = runEnd(
        () -> setMotorSpeed(ClimberConstant.kClimbDownSpeed), this::stopMotor);
    cmd.setName("climbDown");
    return cmd;
  }
}
