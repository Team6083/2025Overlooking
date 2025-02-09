// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.Rev2mDistanceSensor.Port;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CoralShooterConstant;
import frc.robot.lib.DistanceSensor;
import frc.robot.lib.DistanceSensorInterface;

public class CoralShooterSubsystem extends SubsystemBase {
  /** Creates a new CoralShooterSubsystem. */

  private VictorSPX coralShooterLeftMotor;
  private VictorSPX coralShooterRightMotor;
  private DistanceSensorInterface distanceSensor;

  public CoralShooterSubsystem() {
    coralShooterLeftMotor = new VictorSPX(CoralShooterConstant.kShooterLeftMotorChannel);
    coralShooterRightMotor = new VictorSPX(CoralShooterConstant.kShooterRightMotorChannel);
    distanceSensor = new DistanceSensor(Port.kOnboard);
    coralShooterRightMotor.setInverted(CoralShooterConstant.kCoralShooterMotorInverted);
    coralShooterLeftMotor.setInverted(CoralShooterConstant.kCoralShooterMotorInverted);
  }

  private void setMotorSpeed(double speed) {
    coralShooterLeftMotor.set(VictorSPXControlMode.PercentOutput, speed);
    coralShooterRightMotor.set(VictorSPXControlMode.PercentOutput, -speed);
  }

  public void coralShooterFastOn() { // Motor on Fast
    setMotorSpeed(CoralShooterConstant.kShooterMotorFastSpeed);
  }

  public void coralShooterSlowOn() { // Motor on Slow
    setMotorSpeed(CoralShooterConstant.kShooterMotorSlowSpeed);
  }

  public void coralShooterStop() { // Motor stop
    setMotorSpeed(0.0);
  }

  public boolean isGetTarget() {
    if (distanceSensor.isGetTarget()) {
      return distanceSensor.getTargetDistance() <= CoralShooterConstant.kDistanceRange
          && distanceSensor.getTargetDistance() > 0;
    }
    return false;
  }

  public Command coralShooterFastOnCmd() { // Motor on
    Command cmd = runEnd(this::coralShooterFastOn, this::coralShooterStop);
    return cmd;
  }

  public Command coralShooterSlowOnCmd() {
    Command cmd = runEnd(this::coralShooterSlowOnCmd, this::coralShooterStop);
    return cmd;
  }

}