// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.Rev2mDistanceSensor.Port;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CoralShooterConstant;
import frc.robot.lib.DistanceSensor;
import frc.robot.lib.DistanceSensorInterface;

public class CoralShooterSubsystem extends SubsystemBase {
  /** Creates a new CoralShooterSubsystem. */

  private VictorSPX coralShooterLeftMotor;
  private VictorSPX coralShooterRightMotor;
  private Encoder coralShooterEncoder;
  private DistanceSensorInterface distanceSensor;

  public CoralShooterSubsystem() {
    coralShooterLeftMotor = new VictorSPX(CoralShooterConstant.kShooterLeftMotorChannel);
    coralShooterRightMotor = new VictorSPX(CoralShooterConstant.kShooterRightMotorChannel);
    coralShooterEncoder = new Encoder(
        CoralShooterConstant.kShooterEncoderChannelA, CoralShooterConstant.kShooterEncoderChannelB);
    distanceSensor = new DistanceSensor(Port.kOnboard);
    coralShooterRightMotor.setInverted(CoralShooterConstant.kCoralShooterMotorInverted);
    coralShooterLeftMotor.setInverted(CoralShooterConstant.kCoralShooterMotorInverted);
  }

  private void setMotorSpeed(double speed) {
    coralShooterLeftMotor.set(VictorSPXControlMode.PercentOutput, speed);
    coralShooterRightMotor.set(VictorSPXControlMode.PercentOutput, -speed);
  }

  public void coralShooterOn() { // Motor on
    setMotorSpeed(CoralShooterConstant.kShooterMotorSpeed);
  }

  public void coralShooterStop() { // Motor stop
    setMotorSpeed(0.0);
  }

  public boolean isGetCoral() {
    if (distanceSensor.isGetTarget()) {
      return distanceSensor.getTargetDistance() <= CoralShooterConstant.kDistanceRange
          && distanceSensor.getTargetDistance() > 0;
    }
    return false;
  }

  public void encoderReset() {
    coralShooterEncoder.reset();
  }

  public boolean isEnoughRotate() {
    int rotate;
    rotate = coralShooterEncoder.get() / 2048;
    rotate *= 360;
    return rotate >= 180;
  }

  public Command coralShooterShootOnCmd() { // Motor on
    Command cmd = runEnd(this::coralShooterOn, this::coralShooterStop);
    return cmd;
  }

}