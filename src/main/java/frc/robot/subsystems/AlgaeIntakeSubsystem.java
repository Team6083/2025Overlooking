// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Millimeters;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlgaeIntakeConstant;
import frc.robot.lib.Powerdistribution;

public class AlgaeIntakeSubsystem extends SubsystemBase {
  /** Creates a new ALGAEIntakeSubsystem. */
  private final VictorSP intakeMotor;
  private final VictorSP rotateIntakeMotor;
  private final PIDController algaeMotorUpPID;
  private final PIDController algaeMotorDownPID;
  private final Encoder algaeEncoder;
  private final Encoder algaeRotateEncoder;
  private final PowerDistribution powerDistribution;

  public AlgaeIntakeSubsystem(PowerDistribution powerDistribution) {
    this.powerDistribution = powerDistribution;
    algaeMotorUpPID = new PIDController(0, 0, 0);
    algaeMotorDownPID = new PIDController(0, 0, 0);
    intakeMotor = new VictorSP(AlgaeIntakeConstant.kIntakeMotorChannel);
    rotateIntakeMotor = new VictorSP(AlgaeIntakeConstant.kIntakeRotateMotorChannal);
    intakeMotor.setInverted(AlgaeIntakeConstant.kIntakeMotorInverted);
    algaeEncoder = new Encoder(0, 1);
    algaeRotateEncoder = new Encoder(0, 0);
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

  public void setIntakeMotor() { 
    if(powerDistribution.isAlgaeIntakeOverCurrent()){
      intakeMotor.setVoltage(0);
      
    }
    intakeMotor.setVoltage(AlgaeIntakeConstant.kIntakeVoltage);
  }

  public void setReIntake() {
    intakeMotor.setVoltage(AlgaeIntakeConstant.kReIntakeVoltage);
  }

  public void setUpIntake() { 
    rotateIntakeMotor.setVoltage(AlgaeIntakeConstant.kUpIntakeRotateVoltage);
    rotateIntakeMotor.set(algaeMotorUpPID.calculate(algaeRotateEncoder.get()));
  }


  public void setDownIntake() {
    rotateIntakeMotor.setVoltage(AlgaeIntakeConstant.kDownIntakeRotateVoltage);
    rotateIntakeMotor.set(algaeMotorDownPID.calculate(algaeRotateEncoder.get()));
  }

  public void stopRotateIntakeMotor() {
    rotateIntakeMotor.setVoltage(0);
  }

  public double getIntakeMotorRotate() {
    return algaeRotateEncoder.get();
  }

  public double getIntakeShooter() {
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
    // This method will be called once per scheduler run
   

  }
}
