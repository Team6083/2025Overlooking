// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.Rev2mDistanceSensor.Port;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CoralShooterConstant;
import frc.robot.PreferencesClass.CoralShooter;
import frc.robot.lib.sensor.distance.Rev2mDistanceSensor;

public class CoralShooterSubsystem extends SubsystemBase {
  /** Creates a new CoralShooterSubsystem. */
  private VictorSPX coralShooterMotor;
  private Rev2mDistanceSensor distanceSensor;
  private DutyCycleEncoder shooterEncoder;

  public CoralShooterSubsystem() {
    coralShooterMotor = new VictorSPX(CoralShooterConstant.kShooterMotorChannel);
    coralShooterMotor.setInverted(CoralShooterConstant.kCoralShooterMotorInverted);

    shooterEncoder = new DutyCycleEncoder(
        CoralShooterConstant.kShooterEncoderChannel,
        CoralShooterConstant.kEncoderFullRange,
        CoralShooterConstant.kEncoderOffset);
    shooterEncoder.setInverted(CoralShooterConstant.kCoralShooterEncoderInverted);

    distanceSensor = new Rev2mDistanceSensor(Port.kOnboard);
    distanceSensor.setAutomaticMode(true);
  }

  public double getEncoder() {
    return shooterEncoder.get();
  }

  public void setMotorSpeed(double speed) {
    coralShooterMotor.set(ControlMode.PercentOutput, speed);
  }

  public void coralShooterIn() {
    setMotorSpeed(CoralShooter.get("kCoralInSpeed"));
  }

  public void coralShooterOut() {
    setMotorSpeed(CoralShooter.get("kCoralOutSpeed"));
  }

  public void coralShooterReverseShoot() {
    setMotorSpeed(CoralShooter.get("kReverseSpeed"));
  }

  public void coralShooterStop() { // Motor stop
    setMotorSpeed(0.0);
  }

  public boolean isGetTarget() {
    if (distanceSensor.getDistance() <= CoralShooterConstant.kDistanceRange
        && distanceSensor.getDistance() > 0) {
      return true;
    }
    return false;
  }

  public Command coralShooterInCmd() {
    Command cmd = runEnd(this::coralShooterIn, this::coralShooterStop);
    cmd.setName("coralShooterIn");
    return cmd;
  }

  public Command coralShooterOutCmd() {
    Command cmd = runEnd(this::coralShooterOut, this::coralShooterStop);
    cmd.setName("coralShooterOut");
    return cmd;
  }

  public Command coralShooterReverseShootCmd() {
    Command cmd = runEnd(this::coralShooterReverseShoot, this::coralShooterStop);
    cmd.setName("coralShooterReverseOn");
    return cmd;
  }

  public Command coralShooterStopCmd() {
    Command cmd = runOnce(this::coralShooterStop);
    cmd.setName("coralShooterStop");
    return cmd;
  }

  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Distance", distanceSensor.getDistance());
    SmartDashboard.putBoolean("IsGetTarget", isGetTarget());
    SmartDashboard.putNumber("CoralShooterEncoder", shooterEncoder.get());
  }
}