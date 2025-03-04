// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.Rev2mDistanceSensor;
import com.revrobotics.Rev2mDistanceSensor.Port;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CoralShooterConstant;
import frc.robot.lib.PowerDistribution;

public class CoralShooterSubsystem extends SubsystemBase {
  /** Creates a new CoralShooterSubsystem. */
  private VictorSPX coralShooterMotor;
  private Rev2mDistanceSensor distanceSensor;
  private PowerDistribution powerDistribution;
  private DutyCycleEncoder shooterEncoder;

  public CoralShooterSubsystem(PowerDistribution powerDistribution) {
    this.powerDistribution = powerDistribution;
    coralShooterMotor = new VictorSPX(CoralShooterConstant.kShooterMotorChannel);
    shooterEncoder = new DutyCycleEncoder(
        CoralShooterConstant.kShooterEncoderChannel,
        CoralShooterConstant.kEncoderFullRange,
        CoralShooterConstant.kEncoderOffset);
    distanceSensor = new Rev2mDistanceSensor(Port.kOnboard);
    coralShooterMotor.setInverted(CoralShooterConstant.kCoralShooterMotorInverted);
    shooterEncoder.setInverted(CoralShooterConstant.kCoralShooterEncoderInverted);
  }

  public double getEncoder() {
    return shooterEncoder.get();
  }

  public void setMotorSpeed(double speed) {

    if (powerDistribution.isCoralShooterOverCurrent()) {
      setMotorSpeed(0);
      return;
    }
    coralShooterMotor.set(VictorSPXControlMode.PercentOutput, speed);
  }

  // shooter on
  public void coralShooterOn() {
    if (powerDistribution.isCoralShooterOverCurrent()) {
      setMotorSpeed(0);
      return;
    }
    setMotorSpeed(CoralShooterConstant.kShooterMotorSpeed);
  }

  public void coralShooterStop() {
    setMotorSpeed(0.0);
  }

  public boolean isGetTarget() {
    if (distanceSensor.getRange() <= CoralShooterConstant.kDistanceRange
        && distanceSensor.getRange() > 0) {
      return true;
    }
    return false;
  }

  public Command coralShooterOnCmd() {
    Command cmd = runEnd(this::coralShooterOn, this::coralShooterStop);
    cmd.setName("coralShooterOn");
    return cmd;
  }

  public Command coralShooterStopCmd() {
    Command cmd = runOnce(this::coralShooterStop);
    cmd.setName("coralShooterStop");
    return cmd;
  }

  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Distance", distanceSensor.getRange());
    SmartDashboard.putBoolean("IsGetTarget", isGetTarget());
    SmartDashboard.putNumber("CoralShooterEncoder", shooterEncoder.get());
    distanceSensor.setAutomaticMode(true);
  }

}