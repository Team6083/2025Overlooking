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
  private PowerDistribution powerDistribution;
  private Rev2mDistanceSensor distanceSensor;
  private VictorSPX coralShooterMotor;
  private DutyCycleEncoder coralShooterEncoder;

  public CoralShooterSubsystem(PowerDistribution powerDistribution) {
    this.powerDistribution = powerDistribution;

    // motor
    coralShooterMotor = new VictorSPX(CoralShooterConstant.kMotorChannel);
    coralShooterMotor.setInverted(CoralShooterConstant.kMotorInverted);

    // encoder
    coralShooterEncoder = new DutyCycleEncoder(
        CoralShooterConstant.kEncoderChannel,
        CoralShooterConstant.kEncoderFullRange,
        CoralShooterConstant.kEncoderOffset);
    coralShooterEncoder.setInverted(CoralShooterConstant.kEncoderInverted);

    // distance sensor
    distanceSensor = new Rev2mDistanceSensor(Port.kOnboard);
  }

  public void periodic() {
    // put dashboard
    SmartDashboard.putNumber("Distance", distanceSensor.getRange());
    SmartDashboard.putBoolean("IsGetTarget", isGetTarget());
    SmartDashboard.putNumber("CoralShooterEncoder", coralShooterEncoder.get());
    
    distanceSensor.setAutomaticMode(true);
  }

  public double getEncoder() {
    return coralShooterEncoder.get();
  }

  public void setMotorSpeed(double speed) {
    coralShooterMotor.set(VictorSPXControlMode.PercentOutput, speed);
  }

  // shooter on
  public void coralShooterOn() {
    if (powerDistribution.isCoralShooterOverCurrent()) {
      setMotorSpeed(0);
      return;
    }

    setMotorSpeed(CoralShooterConstant.kMotorSpeed);
  }

  public void coralShooterStop() {
    setMotorSpeed(0);
  }

  public boolean isGetTarget() {
    if (distanceSensor.getRange() <= CoralShooterConstant.kDistanceRange && distanceSensor.getRange() > 0) {
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
}