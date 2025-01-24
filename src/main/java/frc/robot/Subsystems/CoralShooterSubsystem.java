// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems ;

import com.revrobotics.Rev2mDistanceSensor;
import com.revrobotics.Rev2mDistanceSensor.Port;

import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CoralShooter;
import frc.robot.lib.DistanceSensor;
import frc.robot.lib.DistanceSensorInterface;
import frc.robot.lib.SimDistanceSensor;

public class CoralShooterSubsystem extends SubsystemBase {
  /** Creates a new CoralShooterSubsystem. */

  private VictorSP CoralShooterMotor;
  private DistanceSensorInterface distanceSensor;
  private double Vol;

  public CoralShooterSubsystem() { 
    CoralShooterMotor = new VictorSP(CoralShooter.kShooterMotorChannel);
    distanceSensor = new DistanceSensor(Port.kOnboard);
  }

  private void SetMotorVoltage(double Vol) { // 設定電壓
    this.Vol = Vol;
    CoralShooterMotor.set(Vol);
  }

  public void CoralShooterOn() { // 正轉
    SetMotorVoltage(CoralShooter.kShooterMotorSpeed);
  }

  public void CoralShooterStop() { // 停止
    SetMotorVoltage(0.0);
  }

  public boolean isGetCoral() {
    if (distanceSensor.isGetTarget()) { // 碰到目標
      return distanceSensor.getTargetDistance() <= CoralShooter.kDistanceRange
          && distanceSensor.getTargetDistance() > 0;
    }
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public Command CoralShooterShootOncmd() { // 正轉
    Command cmd = runEnd(this::CoralShooterOn, this::CoralShooterStop);
    return cmd;
  }

  public Command CoralShooterStopcmd() { // 停止
    Command cmd = runOnce(this::CoralShooterStop);
    return cmd;
  }

}