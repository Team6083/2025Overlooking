// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlgaeIntakeConstant;

public class AlgaeIntakeSubsystem extends SubsystemBase {
  /** Creates a new ALGAEIntakeSubsystem. */
  private final VictorSP intakeMotor;
  private final VictorSP RotateIntakeMotor;
  private final Timer timer;

  public AlgaeIntakeSubsystem() {

    intakeMotor = new VictorSP(AlgaeIntakeConstant.kIntakeMotorChannel);
    RotateIntakeMotor = new VictorSP(AlgaeIntakeConstant.kRotateMotorChannel);
    timer = new Timer();
  }

  public void setTimer() {
    timer.start();
    if (timer.get() > 8) {
      intakeMotor.setVoltage(AlgaeIntakeConstant.kIntakeVoltage);
    }
  }

  public void setIntakeMotor() {
    intakeMotor.setVoltage(AlgaeIntakeConstant.kIntakeVoltage);
  }

  public void setReIntake() {
    intakeMotor.setVoltage(AlgaeIntakeConstant.kReIntakeVoltage);
  }

  public void setUpIntake() {
    RotateIntakeMotor.setVoltage(AlgaeIntakeConstant.kUpIntakeVoltage);
  }

  public void setDownIntake() {
    RotateIntakeMotor.setVoltage(AlgaeIntakeConstant.kDownIntakeVoltage);
  }

  public void stopRotateIntakeMotor() {
    RotateIntakeMotor.setVoltage(0);
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
    // This method will be called once per scheduler run

  }
}
