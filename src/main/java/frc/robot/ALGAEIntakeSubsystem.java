// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ALGAEIntakeSubsystem extends SubsystemBase {
  /** Creates a new ALGAEIntakeSubsystem. */
  private final VictorSP intakeMotor;
  private final VictorSP RotateIntakeMotor;
  private final PIDController algaeMotorUpPID;
  private final PIDController algaeMotorDownPID;
  private final Encoder algaeEncoder;

  public ALGAEIntakeSubsystem() {

    intakeMotor = new VictorSP(0);
    RotateIntakeMotor = new VictorSP(1);
    algaeMotorUpPID = new PIDController(0.8, 0, 0);
    algaeMotorDownPID = new PIDController(0.8, 0, 0);
    algaeEncoder = new Encoder(1, 0);
  }

  public void setIntakeMotor() {
    intakeMotor.setVoltage(6.0);
  }

  public void setReIntake() {
    intakeMotor.setVoltage(-3);
  }

  public void stopRotateIntakeMotor() {
    RotateIntakeMotor.setVoltage(0);
  }

  public void setUpIntake() {
    RotateIntakeMotor.set(algaeMotorUpPID.calculate(algaeEncoder.get()));
  }

  public void setDownIntake() {
    RotateIntakeMotor.set(algaeMotorDownPID.calculate(algaeEncoder.get()));
  }

  public double getUpIntakeSetpoint() {
    return algaeMotorUpPID.getSetpoint();
  }

  public double getDownIntakeSetpoint() {
    return algaeMotorUpPID.getSetpoint();
  }

  public void setUpIntakeSetpoint() {
    algaeMotorUpPID.setSetpoint(130);
  }

  public void setDownIntakeSetpoint() {
    algaeMotorDownPID.setSetpoint(35);
  }

  public double getIntakeMotorRotate() {
    return algaeEncoder.get();
  }

  public Command setUpIntakeCmd() {
    Command cmd = runEnd(this::setUpIntake, this::stopRotateIntakeMotor);
    return cmd;
  }

  public Command setDownIntakeCmd() {
    Command cmd = runEnd(this::setDownIntake, this::stopRotateIntakeMotor);
    return cmd;
  }

  public Command setIntakeMotorCmd() {
    Command cmd = runEnd(this::setUpIntake, this::stopRotateIntakeMotor);
    return cmd;
  }

  public Command setReIntakeMotorCmd() {
    Command cmd = runEnd(this::setUpIntake, this::stopRotateIntakeMotor);
    return cmd;
  }

  @Override
  public void periodic() {

  }
}
