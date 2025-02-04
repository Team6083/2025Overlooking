// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Encoder;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlgaeIntakeConstant;

public class AlgaeIntakeSubsystem extends SubsystemBase {
  /** Creates a new ALGAEIntakeSubsystem. */
  private final VictorSP intakeMotor;
  private final PIDController algaeMotorUpPID;
  private final PIDController algaeMotorDownPID;
  private final Encoder algaeEncoder;

  public AlgaeIntakeSubsystem() {

    intakeMotor = new VictorSP(AlgaeIntakeConstant.kIntakeMotorChannel);
    algaeMotorUpPID = new PIDController(AlgaeIntakeConstant.UpMotorPIDkP, AlgaeIntakeConstant.UpMotorPIDkI,
        AlgaeIntakeConstant.UpMotorPIDkD);
    algaeMotorDownPID = new PIDController(AlgaeIntakeConstant.DownMotorPIDkP, AlgaeIntakeConstant.DownMotorPIDkI,
        AlgaeIntakeConstant.DownMotorPIDkD);
    algaeEncoder = new Encoder(AlgaeIntakeConstant.kalgaeEncoderChannelA, AlgaeIntakeConstant.kalgaeEncoderChannelB);
  }

  public void setIntakeMotor() {
    intakeMotor.setVoltage(AlgaeIntakeConstant.kIntakeVoltage);
  }

  public void setReIntake() {
    intakeMotor.setVoltage(AlgaeIntakeConstant.kReIntakeVoltage);
  }

  public void stopIntakeMotor() {
    intakeMotor.setVoltage(0);
  }

  public double getUpIntakeSetpoint() {
    return algaeMotorUpPID.getSetpoint();
  }

  public double getDownIntakeSetpoint() {
    return algaeMotorUpPID.getSetpoint();
  }

  public void setUpIntakeSetpoint() {
    algaeMotorUpPID.setSetpoint(AlgaeIntakeConstant.kUpIntakeSetpoint);
  }

  public void setDownIntakeSetpoint() {
    algaeMotorDownPID.setSetpoint(AlgaeIntakeConstant.kDownIntakeSetpoint);
  }

  public double getIntakeMotorRotate() {
    return algaeEncoder.get();
  }

  public Command setIntakeMotorCmd() {
    Command cmd = runEnd(this::setIntakeMotor, this::stopIntakeMotor);
    return cmd;
  }

  public Command setReIntakeMotorCmd() {
    Command cmd = runEnd(this::setReIntake, this::stopIntakeMotor);
    return cmd;
  }

  @Override
  public void periodic() {

  }
}
