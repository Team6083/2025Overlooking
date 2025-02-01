// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.Rev2mDistanceSensor.Port;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CoralShooterConstant;
import frc.robot.lib.DistanceSensor;
import frc.robot.lib.DistanceSensorInterface;

public class CoralShooterSubsystem extends SubsystemBase {
  /** Creates a new CoralShooterSubsystem. */

  private VictorSP coralShooterLeftMotor;
  private VictorSP coralShooterRightMotor;
  private DistanceSensorInterface distanceSensor;

  public CoralShooterSubsystem() {
    coralShooterLeftMotor = new VictorSP(CoralShooterConstant.kShooterLeftMotorChannel);
    coralShooterRightMotor = new VictorSP(CoralShooterConstant.kShooterRightMotorChannel);
    distanceSensor = new DistanceSensor(Port.kOnboard);
  }

  private void setMotorVoltage(double voltage) { // 設定電壓
    coralShooterLeftMotor.setVoltage(voltage);
    coralShooterRightMotor.setVoltage(-voltage);
  }

  public void coralShooterOn() { // 正轉
    setMotorVoltage(CoralShooterConstant.kShooterMotorVoltage);
  }

  public void coralShooterStop() { // 停止
    setMotorVoltage(0.0);
  }

  public boolean isGetCoral() {
    if (distanceSensor.isGetTarget()) { // 碰到目標
      return distanceSensor.getTargetDistance() <= CoralShooterConstant.kDistanceRange
          && distanceSensor.getTargetDistance() > 0;
    }
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public Command coralShooterShootOnCmd() { // 正轉
    Command cmd = runEnd(this::coralShooterOn, this::coralShooterStop);
    return cmd;
  }

}