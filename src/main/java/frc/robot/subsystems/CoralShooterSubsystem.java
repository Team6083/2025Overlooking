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
  private VictorSPX coralShooterLeftMotor;
  private VictorSPX coralShooterRightMotor;
  private Rev2mDistanceSensor distanceSensor;
  private PowerDistribution powerDistribution;
  private DutyCycleEncoder shooterEncoder;

  public CoralShooterSubsystem(PowerDistribution powerDistribution) {
    this.powerDistribution = powerDistribution;
    coralShooterLeftMotor = new VictorSPX(CoralShooterConstant.kShooterLeftMotorChannel);
    coralShooterRightMotor = new VictorSPX(CoralShooterConstant.kShooterRightMotorChannel);
    shooterEncoder = new DutyCycleEncoder(
      CoralShooterConstant.kShooterEncoderChannel,
      CoralShooterConstant.kEncoderFullRange,
      CoralShooterConstant.kEncoderOffset);
    distanceSensor = new Rev2mDistanceSensor(Port.kOnboard);
    coralShooterRightMotor.setInverted(CoralShooterConstant.kCoralShooterRightMotorInverted);
    coralShooterLeftMotor.setInverted(CoralShooterConstant.kCoralShooterLeftMotorInverted);
  }

  public double getEncoder() {
    return shooterEncoder.get(); 
  }

  public void setMotorSpeed(double speed) {
    coralShooterLeftMotor.set(VictorSPXControlMode.PercentOutput, speed);
    coralShooterRightMotor.set(VictorSPXControlMode.PercentOutput, speed);
  }

  public void coralShooterFastOn() { // Motor on Fast
    if (powerDistribution.isCoralShooterOverCurrent()) {
      setMotorSpeed(0);
      return;
    }
    setMotorSpeed(CoralShooterConstant.kShooterMotorFastSpeed);
  }

  public void coralShooterSlowOn() { // Motor on Slow
    if (powerDistribution.isCoralShooterOverCurrent()) {
      setMotorSpeed(0);
      return;
    }
    setMotorSpeed(CoralShooterConstant.kShooterMotorSlowSpeed);
  }

  public void CoralShooterAutoIn() {
    while (!isGetTarget()) {
      setMotorSpeed(CoralShooterConstant.kShooterMotorSlowSpeed);
    }
  }

  public void CoralShooterAutoStop() {
    while (isGetTarget()) {
      setMotorSpeed(CoralShooterConstant.kShooterMotorSlowSpeed);
    }
  }

  public void coralShooterStop() { // Motor stop
    setMotorSpeed(0.0);
  }

  public boolean isGetTarget() {
    if (distanceSensor.getRange() <= CoralShooterConstant.kDistanceRange
        && distanceSensor.getRange() > 0) {
      return true;
    }
    return false;
  }

  public Command coralShooterFastOnCmd() { // Motor on
    Command cmd = runEnd(this::coralShooterFastOn, this::coralShooterStop);
    cmd.setName("coralShooterFastOn");
    return cmd;
  }

  public Command coralShooterSlowOnCmd() {
    Command cmd = runEnd(this::coralShooterSlowOn, this::coralShooterStop);
    cmd.setName("coralShooterSlowOn");
    return cmd;
  }

  public Command coralShooterAutoStopCmd() {
    Command cmd = runEnd(this::CoralShooterAutoIn, this::CoralShooterAutoStop);
    cmd.setName("coralShooterAutoStop");
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
    SmartDashboard.putBoolean("isGetTarget", isGetTarget());
    distanceSensor.setAutomaticMode(true);
  }

}